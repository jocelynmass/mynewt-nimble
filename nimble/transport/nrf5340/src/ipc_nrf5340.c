/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <errno.h>
#include <os/os.h>
#include <ipc_nrf5340/ipc_nrf5340.h>
#include <nrfx.h>
#include <nimble/transport/hci_ipc.h>
#include <syscfg/syscfg.h>

#define IPC_MAX_CHANS   MYNEWT_VAL_IPC_NRF5340_CHANNELS
#define IPC_BUF_SIZE    MYNEWT_VAL_IPC_NRF5340_BUF_SZ

struct ipc_shm {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t buf_size;
    uint8_t *buf;
};

/**
 * Initialization structure passed from APP core to NET core.
 * Keeps various parameters that otherwise should be configured on
 * both sides.
 */
struct ipc_shared {
    /** Number of IPC channels */
    uint8_t ipc_channel_count;
    /* Array of shared memories used for IPC */
    struct ipc_shm *ipc_shms;
    /* Set by netcore during IPC initialization */
    volatile enum {
        APP_WAITS_FOR_NET,
        APP_AND_NET_RUNNING,
        NET_RESTARTED,
    } ipc_state;
    volatile struct hci_ipc_shm hci_shm;
};

/* Currently this allows only for 1-1 connection. */

struct ipc_channel {
    ipc_nrf5340_recv_cb cb;
    void *user_data;
};

static struct ipc_channel ipcs[IPC_MAX_CHANS];

#define NET_CRASH_CHANNEL   15

__attribute__ ((section(".ipc"))) static struct ipc_shared ipc_shared[1];

 #if defined(NRF_APPLICATION)
__attribute__ ((section(".ipc"))) static struct ipc_shm shms[IPC_MAX_CHANS];
__attribute__ ((section(".ipc"))) static uint8_t shms_bufs[IPC_MAX_CHANS][IPC_BUF_SIZE];

static void (*net_core_restart_cb)(void);

void
ipc_nrf5340_set_net_core_restart_cb(void (*on_restart)(void))
{
    net_core_restart_cb = on_restart;
}

/* Always use unsecure peripheral for IPC, unless pre-TrusZone bootloader is present in netcore */
#undef NRF_IPC
#define NRF_IPC NRF_IPC_NS

#else
static struct ipc_shm *shms;
#undef IPC_MAX_CHANS
#undef IPC_BUF_SIZE
#define IPC_MAX_CHANS ipc_shared->ipc_channel_count
#define IPC_BUF_SIZE ipc_shared->ipc_shms->buf_size
#endif

static uint16_t
ipc_nrf5340_shm_get_data_length(uint16_t head, uint16_t tail)
{
    return ((unsigned int)(head - tail)) % IPC_BUF_SIZE;
}

/* this function assumes that there is enough space for data */
static void
ipc_nrf5340_shm_write(struct ipc_shm *shm, const void *data, uint16_t data_len)
{
    uint16_t head = shm->head;
    uint16_t len;

    len = min(data_len, IPC_BUF_SIZE - head);
    memcpy(shm->buf + head, data, len);

    /* copy second fragment */
    if (data_len > len) {
        memcpy(shm->buf, data + len, data_len - len);
    }

    head += data_len;
    head %= IPC_BUF_SIZE;

    shm->head = head;
}

static uint16_t
ipc_nrf5340_shm_read(struct ipc_shm *shm, void *buf, struct os_mbuf *om,
                     uint16_t len)
{
    uint16_t head = shm->head;
    uint16_t tail = shm->tail;
    uint16_t frag_len;

    len = min(ipc_nrf5340_shm_get_data_length(head, tail), len);
    if (len == 0) {
        return 0;
    }

    if (buf || om) {
        if (head >= tail) {
            if (buf) {
                memcpy(buf, shm->buf + tail, len);
            } else {
                os_mbuf_append(om, shm->buf + tail, len);
            }
        } else {
            /* in this case we may need to copy two fragments */
            frag_len = min(len, IPC_BUF_SIZE - tail);
            if (buf) {
                memcpy(buf, shm->buf + tail, frag_len);
            } else {
                os_mbuf_append(om, shm->buf + tail, frag_len);
            }

            if (frag_len < len) {
                assert(tail + frag_len == IPC_BUF_SIZE);
                assert(len - frag_len <= head);
                if (buf) {
                    memcpy(buf + frag_len, shm->buf, len - frag_len);
                } else {
                    os_mbuf_append(om, shm->buf, len - frag_len);
                }
            }
        }
    }

    tail += len;
    tail %= IPC_BUF_SIZE;

    shm->tail = tail;

    return len;
}

void
IPC_IRQHandler(void)
{
    uint32_t irq_pend;
    int i;

    /* Handle only interrupts that were enabled */
    irq_pend = NRF_IPC->INTPEND & NRF_IPC->INTEN;

 #if defined(NRF_APPLICATION)
    if (ipc_shared->ipc_state == NET_RESTARTED && (irq_pend & (1UL << NET_CRASH_CHANNEL))) {
        NRF_IPC->EVENTS_RECEIVE[NET_CRASH_CHANNEL] = 0;
        if (net_core_restart_cb) {
            net_core_restart_cb();
        }
        ipc_shared->ipc_state = APP_AND_NET_RUNNING;
    }
#endif

    for (i = 0; i < IPC_MAX_CHANS; i++) {
        if (irq_pend & (0x1UL << i)) {
            NRF_IPC->EVENTS_RECEIVE[i] = 0;
            ipcs[i].cb(i, ipcs[i].user_data);
        }
    }
}

static void
ipc_nrf5340_init_nrf_ipc(void)
{
    int i;

    /* Enable IPC channels */
    for (i = 0; i < IPC_MAX_CHANS; i++) {
        NRF_IPC->SEND_CNF[i] = (0x01UL << i);
        NRF_IPC->RECEIVE_CNF[i] = 0;
    }

    NRF_IPC->INTENCLR = 0xFFFF;
    NVIC_ClearPendingIRQ(IPC_IRQn);
    NVIC_SetPriority(IPC_IRQn, 7);
    NVIC_EnableIRQ(IPC_IRQn);
}

 #if defined(NRF_APPLICATION)
void
ipc_nrf5340_init(void)
{
    int i;

    /* Make sure network core if off when we set up IPC */
    NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;

    memset(ipc_shared, 0, sizeof(ipc_shared));
    memset(shms, 0, sizeof(shms));

    for (i = 0; i < IPC_MAX_CHANS; ++i) {
        shms[i].buf = shms_bufs[i];
        shms[i].buf_size = IPC_BUF_SIZE;
    }
    ipc_shared->ipc_channel_count = IPC_MAX_CHANS;
    ipc_shared->ipc_shms = shms;
    ipc_shared->ipc_state = APP_WAITS_FOR_NET;

    ipc_nrf5340_init_nrf_ipc();
    NRF_IPC->RECEIVE_CNF[NET_CRASH_CHANNEL] = (0x1UL << NET_CRASH_CHANNEL);
    NRF_IPC->INTENSET = (1UL << NET_CRASH_CHANNEL);
}

void
ipc_nrf5340_netcore_init(void)
{
    /* Start Network Core */
    /* Workaround for Errata 161: "RESET: Core is not fully reset after Force-OFF" */
    // *(volatile uint32_t *) ((uint32_t)NRF_RESET_NS + 0x618ul) = 1ul;
    NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
    vTaskDelay(1);
    NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
    vTaskDelay(1);
    NRF_RESET_NS->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Release;
    // *(volatile uint32_t *) ((uint32_t)NRF_RESET_NS + 0x618ul) = 0;

    /*
     * Waits for NET core to start and init it's side of IPC.
     * It may take several seconds if there is net core
     * embedded image in the application flash.
     */
    while (ipc_shared->ipc_state != APP_AND_NET_RUNNING){
        vTaskDelay(1);
    }
}
#endif

#if defined(NRF_NETWORK)
void
ipc_nrf5340_init(void)
{
    /*
     * Wait for application core to pass IPC configuration.
     */
    shms = ipc_shared->ipc_shms;
    assert(ipc_shared->ipc_channel_count <= ARRAY_SIZE(ipcs));

    ipc_nrf5340_init_nrf_ipc();
    NRF_IPC->SEND_CNF[NET_CRASH_CHANNEL] = (0x01UL << NET_CRASH_CHANNEL);
}

void
ipc_nrf5340_netcore_init(void)
{
    /*
     * If ipc_state is already APP_AND_NET_RUNNING it means that net core
     * restarted without app core involvement, notify app core about such
     * case.
     */
    if (ipc_shared->ipc_state == APP_AND_NET_RUNNING) {
        ipc_shared->ipc_state = NET_RESTARTED;
        NRF_IPC->TASKS_SEND[NET_CRASH_CHANNEL] = 1;
        while (ipc_shared->ipc_state == NET_RESTARTED) ;
    } else if (ipc_shared->ipc_state == APP_WAITS_FOR_NET) {
        /* Normal start app core waits for net core to initialize IPC, mark net core as ready */
        ipc_shared->ipc_state = APP_AND_NET_RUNNING;
    }
}
#endif

 #if defined(NRF_APPLICATION)
void
ipc_nrf5340_reset(void)
{
    int i;

    /* TODO  do we need some reset callback for IPC users? */
    /* TODO Should we sync this with send ? */

    /* Make sure network core if off when we reset IPC */
    NRF_RESET->NETWORK.FORCEOFF = RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;

    memset(shms, 0, sizeof(shms));

    for (i = 0; i < IPC_MAX_CHANS; ++i) {
        shms[i].buf = shms_bufs[i];
        shms[i].buf_size = IPC_BUF_SIZE;
    }

    /* Start Network Core */
    ipc_nrf5340_netcore_init();
}
#endif

void
ipc_nrf5340_recv(int channel, ipc_nrf5340_recv_cb cb, void *user_data)
{
    assert(channel < IPC_MAX_CHANS);

    if (cb) {
        assert(ipcs[channel].cb == NULL);

        ipcs[channel].cb = cb;
        ipcs[channel].user_data = user_data;
        NRF_IPC->RECEIVE_CNF[channel] = (0x1UL << channel);
        NRF_IPC->INTENSET = (0x1UL << channel);
    } else {
        NRF_IPC->INTENCLR = (0x1UL << channel);
        NRF_IPC->RECEIVE_CNF[channel] = 0;
        ipcs[channel].cb = NULL;
        ipcs[channel].user_data = NULL;
    }
}

int
ipc_nrf5340_send(int channel, const void *data, uint16_t len)
{
    return ipc_nrf5340_write(channel, data, len, true);
}

int
ipc_nrf5340_write(int channel, const void *data, uint16_t len, bool last)
{
    struct ipc_shm *shm;
    uint16_t frag_len;
    uint16_t space;

    assert(channel < IPC_MAX_CHANS);
    shm = &shms[channel];

    if (data && len) {
        while (len) {
            do {
                space = IPC_BUF_SIZE - 1;
                space -= ipc_nrf5340_shm_get_data_length(shm->head, shm->tail);
#if !MYNEWT_VAL_IPC_NRF5340_BLOCKING_WRITE
                /* assert since that will always fail for non-blocking write
                 * indicating use error
                 */
                assert(len < IPC_BUF_SIZE);
                if (len > space) {
                    return OS_ENOMEM;
                }
#endif
            } while (space == 0);

            frag_len = min(len, space);
            ipc_nrf5340_shm_write(shm, data, frag_len);
            if (last || len > space) {
                NRF_IPC->TASKS_SEND[channel] = 1;
            }

            data += frag_len;
            len -= frag_len;
        }
    }

    return 0;
}

uint16_t
ipc_nrf5340_read(int channel, void *buf, uint16_t len)
{
    assert(channel < IPC_MAX_CHANS);

    return ipc_nrf5340_shm_read(&shms[channel], buf, NULL, len);
}

uint16_t
ipc_nrf5340_read_om(int channel, struct os_mbuf *om, uint16_t len)
{
    assert(channel < IPC_MAX_CHANS);

    return ipc_nrf5340_shm_read(&shms[channel], NULL, om, len);
}

uint16_t
ipc_nrf5340_available(int channel)
{
    return ipc_nrf5340_data_available_get(channel);
}

uint16_t
ipc_nrf5340_available_buf(int channel, void **dptr)
{
    struct ipc_shm *shm = &shms[channel];
    uint16_t head = shm->head;
    uint16_t tail = shm->tail;

    *dptr = &shm->buf[tail];

    if (head > tail) {
        return head - tail;
    } else if (head < tail) {
        return IPC_BUF_SIZE - tail;
    }

    return 0;
}

uint16_t
ipc_nrf5340_data_available_get(int channel)
{
    assert(channel < IPC_MAX_CHANS);

    return ipc_nrf5340_shm_get_data_length(shms[channel].head,
                                           shms[channel].tail);
}

uint16_t
ipc_nrf5340_data_free_get(int channel)
{
    return IPC_BUF_SIZE - ipc_nrf5340_data_available_get(channel) - 1;
}

uint16_t
ipc_nrf5340_consume(int channel, uint16_t len)
{
    assert(channel < IPC_MAX_CHANS);

    return ipc_nrf5340_shm_read(&shms[channel], NULL, NULL, len);
}

volatile struct hci_ipc_shm *
ipc_nrf5340_hci_shm_get(void)
{
    return &ipc_shared->hci_shm;
}