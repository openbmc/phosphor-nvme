
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include "i2c-dev.h"

namespace phosphor
{
namespace smbus
{


    class Smbus
    {
        public:

        Smbus(){};

        void dump_hex(char *outbuf, int len);

        int open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet);

        int set_slave_addr(int file, int address, int force);

        int smbus_block_write(int fd, uint8_t cmd, size_t data_len, uint8_t *data);

        int smbusInit(int smbus_num);

        void smbusClose(int smbus_num);

        int SendSmbusCmd(int smbus_num, int8_t device_addr, uint8_t smbuscmd, char *rsp_data);

        int GetSmbusCmdWord(int smbus_num, int8_t device_addr, int8_t smbuscmd );

        int SetSmbusCmdWord(int smbus_num, int8_t device_addr, int8_t smbuscmd , int16_t data);

        int GetSmbusCmdByte(int smbus_num, int8_t device_addr, int8_t smbuscmd);

        int SetSmbusCmdByte(int smbus_num, int8_t device_addr, int8_t smbuscmd , int8_t data);

        int SmbusCmdProcessCall(int smbus_num, int8_t device_addr, int8_t smbuscmd , int16_t smbusdata );

        int SendSmbusRWBlockCmd(int smbus_num, int8_t device_addr, uint8_t *tx_data,
            uint8_t tx_len, char *rsp_data);

        int SendSmbusWBlockCmd(int smbus_num, int8_t device_addr , int8_t command, uint8_t *tx_data,
            uint8_t tx_len);

        int SendSmbusWBlockByMasterWrite(int smbus_num, int8_t device_addr , int8_t command, uint8_t *tx_data,
            uint8_t tx_len);

        int SendSmbusRWBlockCmdRAW(int smbus_num, int8_t device_addr, uint8_t *tx_data,
            uint8_t tx_len, uint8_t *rsp_data);
    };

}
}