/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

uint8_t led_nr;

nrf_esb_payload_t rx_payload, ack_payload;
volatile bool packet_received;

bool rx_enabled = true;
bool read_rx_payload_enabled = true;

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

static void read_rx_payloads(void)
{
    while(nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
    {
        // Set LEDs identical to the ones on the PTX.
        nrf_gpio_pin_write(LED_1, !(rx_payload.data[1]%8>0 && rx_payload.data[1]%8<=4));
        nrf_gpio_pin_write(LED_2, !(rx_payload.data[1]%8>1 && rx_payload.data[1]%8<=5));
        nrf_gpio_pin_write(LED_3, !(rx_payload.data[1]%8>2 && rx_payload.data[1]%8<=6));
        nrf_gpio_pin_write(LED_4, !(rx_payload.data[1]%8>3));
        NRF_LOG_HEXDUMP_INFO(rx_payload.data, rx_payload.length);
        //NRF_LOG_INFO("Receiving packet: %02x", rx_payload.data[1]);
        packet_received = true;
    }
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            NRF_LOG_INFO("TX SUCCESS EVENT");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            NRF_LOG_INFO("TX FAILED EVENT");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            NRF_LOG_INFO("RX RECEIVED EVENT");
            if(read_rx_payload_enabled)
            {
                read_rx_payloads();
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    bsp_board_init(BSP_INIT_LEDS);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 8;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

static bool button_pressed(int index)
{
    static bool pressed_last[4] = {false, false, false, false};
    bool pressed_now = nrf_gpio_pin_read(BUTTON_1 + index) == 0;
    if(pressed_now && !pressed_last[index])
    {
        pressed_last[index] = true;
        return true;
    }
    pressed_last[index] = pressed_now;
    return false;
}

int main(void)
{
    uint32_t err_code;
    uint8_t counter;
    uint8_t ack_pl_counter = 0;

    nrf_gpio_range_cfg_input(BUTTON_1, BUTTON_4, NRF_GPIO_PIN_PULLUP);

    ack_payload.pipe = 0;
    ack_payload.length = 6;
    for(int i = 0; i < 6; i++) ack_payload.data[i] = i + 0x10;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Enhanced ShockBurst Receiver Example started.");

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        while(NRF_LOG_PROCESS());
        if(false)//packet_received)
        {
            packet_received = false;
            ack_payload.data[1] = counter;
            nrf_esb_write_payload(&ack_payload);
            counter--;
        }
        if(button_pressed(0))
        {
            rx_enabled = !rx_enabled;
            if(rx_enabled)
            {
                NRF_LOG_INFO("RX On");
                APP_ERROR_CHECK(nrf_esb_start_rx());
            }
            else
            {
                NRF_LOG_INFO("RX Off");
                APP_ERROR_CHECK(nrf_esb_stop_rx());
            }
        }
        if(button_pressed(1))
        {
            read_rx_payload_enabled = !read_rx_payload_enabled;
            if(read_rx_payload_enabled)
            {
                NRF_LOG_INFO("RX Read ON");
                read_rx_payloads();
            }
            else
            {
                NRF_LOG_INFO("RX Read Off");
            }
        }
        if(button_pressed(2))
        {
            static uint32_t pipe_counter = 0;
            ack_payload.pipe = pipe_counter;
            ack_payload.length = 8;
            for(int i = 0; i < 8; i++) ack_payload.data[i] = ack_pl_counter;
            if(nrf_esb_write_payload(&ack_payload) == 0)
            {
                NRF_LOG_INFO("Uploading ACK payload with counter %i on pipe %i", ack_pl_counter, pipe_counter);
            }
            else
            {
                NRF_LOG_INFO("ACK payload upload failed");
            }
            pipe_counter = (pipe_counter + 1) % 3;
            ack_pl_counter++;
        }
        //__WFE();
        
    }
}
/*lint -restore */
