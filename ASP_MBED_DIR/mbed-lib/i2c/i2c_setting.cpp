/*
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include "mbed.h"
#include "i2c_setting.h"

#define REG_REQ_BUF_SIZE        (7)
#define DATA_MAX_SIZE           (32)
#define ARG_MAX_NUM             (DATA_MAX_SIZE + 3)      // Reqest, I2C addr, len, data1, data2, data3, ...
#define ARG_MAX_SIZE            (2)                      // upper bit + lower bit
#define NULL_SIZE               (1)
#define CODE_NULL               (0x00)
#define NUM_STR_TO_HEX          (0x30)
#define BIG_STR_TO_HEX          (0x37)
#define SMA_STR_TO_HEX          (0x57)
#define MASK_HEX10              (0x10)

#define OFS_REQ                 (0)
#define OFS_I2C_ADDR            (1)
#define OFS_DATA_SIZE           (2)
#define OFS_DATA                (3)

#define STR_WR                  "Wr:"
#define STR_RD                  "Rd:"
#define STR_WR_NO_P             "WrNoP:"
#define STR_RD_NO_P             "RdNoP:"

#define REQ_NONE                (0)
#define REQ_WR                  (1)
#define REQ_RD                  (2)
#define REQ_WR_NO_P             (3)
#define REQ_RD_NO_P             (4)

I2C i2c(I2C_SDA, I2C_SCL);

static char hex_to_char_tbl[] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F'
};

static int str_to_hex(char * psrcbuf, char * pdestbuf, int cnt) {
    int retval = false;
    int32_t tmp_hex;

    if ((psrcbuf != NULL) && (pdestbuf != NULL)) {
        retval = true;
        if ((((int32_t)*psrcbuf) >= '0') && (((int32_t)*psrcbuf) <= '9')) {
            tmp_hex = NUM_STR_TO_HEX;
        } else if ((((int32_t)*psrcbuf) >= 'A') && (((int32_t)*psrcbuf) <= 'F')) {
            tmp_hex = BIG_STR_TO_HEX;
        } else if ((((int32_t)*psrcbuf) >= 'a') && (((int32_t)*psrcbuf) <= 'f')) {
            tmp_hex = SMA_STR_TO_HEX;
        } else {
            retval = false;
        }
        if (retval == true) {
            *pdestbuf += ((int32_t)*psrcbuf) - tmp_hex;
            if (cnt == 0) {
                *pdestbuf *= MASK_HEX10;
            }
        }
    }

    return retval;
}

static void char_to_16char(char * pdestbuf, char * psrcbuf, int length) {
    if ((pdestbuf != NULL) && (psrcbuf != NULL)) {
        while (1) {
            *pdestbuf = hex_to_char_tbl[((int32_t)*psrcbuf) / MASK_HEX10];
            pdestbuf++;
            *pdestbuf = hex_to_char_tbl[((int32_t)*psrcbuf) % MASK_HEX10];
            pdestbuf++;

            psrcbuf++;
            length--;
            if (length != 0) {
                *pdestbuf = ',';
                pdestbuf++;
            } else {
                break;
            }
        }
        *pdestbuf = CODE_NULL;
    }
}

static int analysis_cmd(char * buf, char * p_reg_arg_buf) {
    int  arg_cnt = 0;
    int  byte_cnt = 0;
    int  retval;
    char * psrcbuf = buf;
    int  ret;

    if (strncmp(psrcbuf, STR_WR, sizeof(STR_WR) - 1) == 0) {
        ret = REQ_WR;
        psrcbuf += sizeof(STR_WR) - 1;
    } else if (strncmp(psrcbuf, STR_RD, sizeof(STR_RD) - 1) == 0) {
        ret = REQ_RD;
        psrcbuf += sizeof(STR_RD) - 1;
    } else if (strncmp(psrcbuf, STR_WR_NO_P, sizeof(STR_WR_NO_P) - 1) == 0) {
        ret = REQ_WR_NO_P;
        psrcbuf += sizeof(STR_WR_NO_P) - 1;
    } else if (strncmp(psrcbuf, STR_RD_NO_P, sizeof(STR_RD_NO_P) - 1) == 0) {
        ret = REQ_RD_NO_P;
        psrcbuf += sizeof(STR_RD_NO_P) - 1;
    } else {
        ret = REQ_NONE;
    }

    if (ret != REQ_NONE) {
        /* get argument(Reqest, I2C addr, len, data1, data2, data3, ...) */
        p_reg_arg_buf[arg_cnt] = ret;
        arg_cnt++;
        byte_cnt = 0;
        while (((int32_t)*psrcbuf) != CODE_NULL) {
            retval = str_to_hex(psrcbuf, &p_reg_arg_buf[arg_cnt], byte_cnt);
            if (retval != false) {
                byte_cnt++;
                if (byte_cnt >= ARG_MAX_SIZE) {
                    if ((arg_cnt + 1) >= ARG_MAX_NUM) {
                        ret = REQ_NONE;
                        break;
                    } else {
                        arg_cnt++;
                        byte_cnt = 0;
                    }
                }
            }
            psrcbuf++;
        }
    }

    return ret;
}

static void execute_cmd(char * buf, char * p_reg_arg_buf) {
    int ret;
    size_t len;
    int stop = 0;

    /* check request */
    if ((p_reg_arg_buf[OFS_REQ] == REQ_WR_NO_P) || (p_reg_arg_buf[OFS_REQ] == REQ_RD_NO_P)) {
        stop = 1;
    }

    switch (p_reg_arg_buf[OFS_REQ]) {
        case REQ_WR:
        case REQ_WR_NO_P:
            ret = i2c.write(p_reg_arg_buf[OFS_I2C_ADDR], &p_reg_arg_buf[OFS_DATA], p_reg_arg_buf[OFS_DATA_SIZE], stop);
            if (ret == 0) {
                sprintf(buf, "OK");
            }
        break;
        case REQ_RD:
        case REQ_RD_NO_P:
            ret = i2c.read(p_reg_arg_buf[OFS_I2C_ADDR], &p_reg_arg_buf[OFS_DATA], p_reg_arg_buf[OFS_DATA_SIZE], stop);
            if (ret == 0) {
                sprintf(buf, "OK ");
                len = strlen(buf);
                char_to_16char(&buf[len], &p_reg_arg_buf[OFS_DATA], p_reg_arg_buf[OFS_DATA_SIZE]);
            }
        break;
        default:
        case REQ_NONE:
            ret = -1;
        break;
    }
    if (ret != 0) {
        sprintf(buf, "NG");
    }
}

bool i2c_setting_exe(char * buf) {
    int reg_arg_cnt;
    char reg_arg_buf[ARG_MAX_NUM] = {0};

    /* analysis command */
    reg_arg_cnt = analysis_cmd(buf, reg_arg_buf);
    if (reg_arg_cnt != REQ_NONE) {
        /* check length */
        if (reg_arg_buf[OFS_DATA_SIZE] >= DATA_MAX_SIZE) {
            reg_arg_buf[OFS_DATA_SIZE] = DATA_MAX_SIZE;
        }
        /* execute command */
        execute_cmd(buf, reg_arg_buf);
        return true;
    }

    return false;
}

#if(0) /* Please enable this line when performing the setting from the Terminal side. */
Serial terminal(USBTX, USBRX);
static char recv_term_buffer[I2C_SETTING_STR_BUF_SIZE];

void SetI2CfromTerm(void const *argument) {
    int32_t term_buf_offset = 0;
    char recv_data;

    while (1) {
        recv_data = terminal.getc();
        /* echo back */
        printf("%c", recv_data);
        switch ((int32_t)recv_data) {
            case 0x0A :
                recv_term_buffer[term_buf_offset] = CODE_NULL;
                term_buf_offset = 0;
                /* command analysis and execute */
                if (i2c_setting_exe(recv_term_buffer) != false) {
                    terminal.puts(recv_term_buffer);
                }
            break;
            case 0x0D :
                /* Do Nothing */
            break;
            default :
                /* check data_buffer size  */
                if (term_buf_offset < I2C_SETTING_STR_BUF_SIZE) {
                    recv_term_buffer[term_buf_offset] = recv_data;
                    term_buf_offset++;
                }
            break;
        }
    }
}
#endif

