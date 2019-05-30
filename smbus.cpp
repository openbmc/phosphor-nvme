/*
    i2c-dev.h - i2c-bus driver, char device interface

    Copyright (C) 1995-97 Simon G. Vogl
    Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include "i2c-dev.h"
#include "smbus.hpp"
#include <iostream>

#include <mutex>

#define SMBUS_PEC     1

/* Transaction Error code */
#define ERR_SMBUS_BLOCK_WRITE    1
#define ERR_SMBUS_BLOCK_READ     2
#define ERR_SEQ_DISMATCH         3

#define MAX_I2C_BUS             30



static int fd[MAX_I2C_BUS] = {0};

// #define printf(format, args...)  {}

namespace phosphor
{
namespace smbus
{

std::mutex gMutex;


void phosphor::smbus::Smbus::dump_hex(char *outbuf, int len)
{
    int i, j;

    printf("\r\n--------------------------------------------------------------------------------\r\n");

    for (i = 0; i < len; i+= 16)
    {
        printf("%4x char ", i);

        for (j = 0; j < 16; j++)
        {
            unsigned char x = outbuf[i+j];

            if (x < 32 || x > 126)
                x = '.';

            if ((i+j) >= len)
                break;

            printf("%c", x);
        }

        printf("%*s hex", 16-j, "");

        for (j = 0; j < 16; j++)
        {
            if ((i+j) >= len)
                break;

            printf(" %02x", (unsigned char)outbuf[i+j]);
        }

        printf("\r\n");
    }

    printf("--------------------------------------------------------------------------------\n");
}


int phosphor::smbus::Smbus::open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet)
{
    int file;

    snprintf(filename, size, "/dev/i2c/%d", i2cbus);
    filename[size - 1] = '\0';
    file = open(filename, O_RDWR);

    if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
        sprintf(filename, "/dev/i2c-%d", i2cbus);
        file = open(filename, O_RDWR);
    }

    if (file < 0 && !quiet) {
        if (errno == ENOENT) {
            fprintf(stderr, "Error: Could not open file "
                "`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
                i2cbus, i2cbus, strerror(ENOENT));
        } else {
            fprintf(stderr, "Error: Could not open file "
                "`%s': %s\n", filename, strerror(errno));
            if (errno == EACCES)
                fprintf(stderr, "Run as root?\n");
        }
    }

    return file;
}

int phosphor::smbus::Smbus::set_slave_addr(int file, int address, int force)
{
    /* With force, let the user read from/write to the registers
       even when a driver is also running */
    if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
        fprintf(stderr,
            "Error: Could not set address to 0x%02x: %s\n",
            address, strerror(errno));
        return -errno;
    }

    return 0;
}

int phosphor::smbus::Smbus::smbus_block_write(int fd, uint8_t cmd, size_t data_len, uint8_t *data)
{
    int res;

    /* Set PEC */
    if (ioctl(fd, I2C_PEC, 1) < 0) {
        fprintf(stderr, "Error: Could not set PEC: %s\n", strerror(errno));
        return -errno;
    }

    /* Smbus block write */
    res = i2c_smbus_write_block_data(fd, cmd, data_len, (__u8 *)data);
    if(res < 0) {
        fprintf(stderr, "Error: i2c_smbus_write_block_data failed\n");
    }

    /* clear PEC */
    if (ioctl(fd, I2C_PEC, 0) < 0) {
        fprintf(stderr, "Error: Could not clear PEC: %s\n", strerror(errno));
        return -errno;
    }

    return res;
}


int phosphor::smbus::Smbus::smbusInit(int smbus_num)
{
    int res = 0;
    char filename[20];

    gMutex.lock();

    fd[smbus_num] = open_i2c_dev(smbus_num, filename, sizeof(filename), 0);
    if(fd[smbus_num] < 0) {
        gMutex.unlock();

        return -1;
    }

    //printf("\rSmbusInit: fd[smbus_num] = %d", fd[smbus_num]);
    res =  fd[smbus_num];

    gMutex.unlock();

    return res;
}

void phosphor::smbus::Smbus::smbusClose(int smbus_num)
{
    close(fd[smbus_num]);
}


int phosphor::smbus::Smbus::SendSmbusCmd(int smbus_num, int8_t device_addr, uint8_t smbuscmd, char *rsp_data)
{
    int res, i, res_len;
    char tmp[4];
    unsigned char Rx_buf[I2C_DATA_MAX] = {0};

    Rx_buf[0] = 1;
    /* Smbus block read */

    gMutex.lock();
    res = i2c_read_after_write(fd[smbus_num], 0, device_addr, 1, (unsigned char *)&smbuscmd, I2C_DATA_MAX, 
        (const unsigned char *)Rx_buf);
    if(res < 0) {
        fprintf(stderr, "Error: SendSmbusCmd failed\n");
        //(smbus_num);
        return -1;
    }

    res_len = Rx_buf[0]+1;

    printf("Rx_buf:");
    for(i = 0; i<res_len;i++)
    {
        printf("%02X", Rx_buf[i]);
    }
    printf("\n");

     /* put rsp data */
    for(i=0; i<res_len; i++) {
        sprintf(tmp, " %02x", Rx_buf[i]);
            strcat(rsp_data, tmp);
    }
    gMutex.unlock();

    return res;
}

int phosphor::smbus::Smbus::GetSmbusCmdWord(int smbus_num, int8_t device_addr, int8_t smbuscmd )
{
    int res;

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);
                //(smbus_num);
            return -1;
        }
    }

    res = i2c_smbus_read_word_data(fd[smbus_num], smbuscmd);
    if (res < 0) {
        fprintf(stderr, "Error: Read failed\n");
        gMutex.unlock();
        return -1;
    }

    printf("\r[GetSmbusCmdWord] 0x%0*x",2, res);
    //(smbus_num);
    return res;
}

int phosphor::smbus::Smbus::SetSmbusCmdWord(int smbus_num, int8_t device_addr, int8_t smbuscmd , int16_t data)
{
    int res;

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);

                gMutex.unlock();
            return -1;
        }
    }

    res = i2c_smbus_write_word_data(fd[smbus_num], smbuscmd, data);
    if (res < 0) {
        fprintf(stderr, "Error: Read failed\n");
        gMutex.unlock();

        return -1;
    }

    printf("\r[GetSmbusCmdWord] 0x%0*x",2, res);
    gMutex.unlock();
    return res;
}

int phosphor::smbus::Smbus::GetSmbusCmdByte(int smbus_num, int8_t device_addr, int8_t smbuscmd)
{
    int res;

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);

                gMutex.unlock();
            return -1;
        }
    }

    res = i2c_smbus_read_byte_data(fd[smbus_num], smbuscmd);
    if (res < 0) {
        fprintf(stderr, "Error: Read failed\n");
        gMutex.unlock();

        return -1;
    }
    // printf("[GetSmbusCmdByte]0x%0*x\n",2, res);

    gMutex.unlock();
    return res;
}

int phosphor::smbus::Smbus::SetSmbusCmdByte(int smbus_num, int8_t device_addr, int8_t smbuscmd , int8_t data)
{
    int res;

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);

                gMutex.unlock();
            return -1;
        }
    }

    res = i2c_smbus_write_byte_data(fd[smbus_num], smbuscmd, data);
    if (res < 0) {
        fprintf(stderr, "Error: Read failed\n");
        gMutex.unlock();

        return -1;
    }
    // printf("[SetSmbusCmdByte]0x%0*x\n",2, res);

    gMutex.unlock();
    return res;
}

int phosphor::smbus::Smbus::SmbusCmdProcessCall(int smbus_num, int8_t device_addr, int8_t smbuscmd , int16_t smbusdata )
{
    int res;

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);

                gMutex.unlock();
            return -1;
        }
    }

    res = i2c_smbus_process_call(fd[smbus_num], smbuscmd , smbusdata);
    if (res < 0) {
        fprintf(stderr, "Error: Read failed\n");

        gMutex.unlock();
        return -1;
    }
    printf("\r[GetSmbusCmdProcessCall]0x%0*x",2, res);

    gMutex.unlock();
    return res;
}


int phosphor::smbus::Smbus::SendSmbusRWBlockCmd(int smbus_num, int8_t device_addr, uint8_t *tx_data,
uint8_t tx_len, char *rsp_data)
{
    int res, i, res_len, tmp_tx_len = 3;
    char tmp[4];
    unsigned char Rx_buf[I2C_DATA_MAX] = {0};
    unsigned char tmp_tx_buf[I2C_DATA_MAX] = {0};

    Rx_buf[0] = 1;
    /* Smbus block read */

    gMutex.lock();
    res = i2c_read_after_write(fd[smbus_num], 0, device_addr, tx_len, (unsigned char *)tx_data, I2C_DATA_MAX, 
        (const unsigned char *)Rx_buf);

    if(res < 0) {
        fprintf(stderr, "Error: SendSmbusCmd failed\n");

        gMutex.unlock();
        return -1;
    }

    res_len = Rx_buf[0]+1;

    // printf("Rx_buf:");
    // for(i = 0; i<res_len;i++)
    // {
    //     printf("%02X", Rx_buf[i]);
    // }
    // printf("\n");

     /* put rsp data */
    for(i=0; i<res_len; i++) {
        sprintf(tmp, " %02x", Rx_buf[i]);
            strcat(rsp_data, tmp);
    }

    gMutex.unlock();
    return res;
}

int phosphor::smbus::Smbus::SendSmbusWBlockCmd(int smbus_num, int8_t device_addr , int8_t command, uint8_t *tx_data,
uint8_t tx_len)
{
    int res, i, res_len, tmp_tx_len = 3;
    char tmp[4];
    unsigned char tmp_tx_buf[I2C_DATA_MAX] = {0};

    gMutex.lock();
    if(fd[smbus_num] > 0) {
        res = set_slave_addr(fd[smbus_num], device_addr, I2C_SLAVE);
        if(res < 0) {
            fprintf(stderr, "set PMBUS BUS%d to slave address 0x%02X failed (%s)\n", smbus_num, device_addr,strerror(errno));
                close(fd[smbus_num]);

                gMutex.unlock();
            return -1;
        }
    }

    /* Set PEC */
    if (ioctl(fd[smbus_num] , I2C_PEC, 1) < 0) {
        fprintf(stderr, "Error: Could not set PEC: %s\n", strerror(errno));

        gMutex.unlock();
        return -errno;
    }


    res = i2c_smbus_write_block_data(fd[smbus_num], command, tx_len, (unsigned char *)tx_data);    

    if(res < 0) {
        fprintf(stderr, "Error: i2c_smbus_write_block_data failed\n");
    }

    /* clear PEC */
    if (ioctl(fd[smbus_num], I2C_PEC, 0) < 0) {
        fprintf(stderr, "Error: Could not clear PEC: %s\n", strerror(errno));

        return -errno;
    }

    return res;
}


int phosphor::smbus::Smbus::SendSmbusWBlockByMasterWrite(int smbus_num, int8_t device_addr , int8_t command, uint8_t *tx_data,
uint8_t tx_len)
{
    int res, i, res_len, tmp_tx_len = 3;
    char tmp[4];
    unsigned char tmp_tx_buf[I2C_DATA_MAX] = {0};

    gMutex.lock();
    res = i2c_master_write(fd[smbus_num],device_addr,tx_len,tx_data);

    if(res < 0) {
        fprintf(stderr, "Error: SendSmbusWBlockByMasterWrite failed\n");
    }


    gMutex.unlock();
    return res;
}


int phosphor::smbus::Smbus::SendSmbusRWBlockCmdRAW(int smbus_num, int8_t device_addr, uint8_t *tx_data,
uint8_t tx_len, uint8_t *rsp_data)
{
    int res, i, res_len, tmp_tx_len = 8;
    char tmp[4];
    unsigned char Rx_buf[I2C_DATA_MAX] = {0};
    unsigned char tmp_tx_buf[I2C_DATA_MAX] = {0};

    Rx_buf[0] = 1;

    gMutex.lock();

    res = i2c_read_after_write(fd[smbus_num], 0, device_addr, tx_len, (unsigned char *)tx_data, I2C_DATA_MAX, 
        (const unsigned char *)Rx_buf);

    if(res < 0) {
        fprintf(stderr, "Error: SendSmbusRWBlockCmdRAW failed\n");
    }

    res_len = Rx_buf[0]+1;

    memcpy(rsp_data,Rx_buf,res_len);

    gMutex.unlock();

    return res;
}

}
}
