/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <malloc.h>
#include <zconf.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

static int el2535_writeu8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
    int wkc;

    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                       EC_TIMEOUTRXM);
    return wkc;
}

static int el2535_writeu16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
    int wkc;

    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                       EC_TIMEOUTRXM);
    return wkc;
}

static int el2535_write16 (uint16 slave, uint16 index, uint8 subindex, int16 value)
{
    int wkc;

    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                       EC_TIMEOUTRXM);
    return wkc;
}

//static int el2535_writeu32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
//{
//    int wkc;
//
//    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
//                       EC_TIMEOUTRXM);
//    return wkc;
//}

static int el2535_write32 (uint16 slave, uint16 index, uint8 subindex, int32 value)
{
    int wkc;

    wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                       EC_TIMEOUTRXM);
    return wkc;
}

static int el2535_setup(uint16 slave)
{
    printf ("el2535 drive setup\n");

    el2535_writeu16  (slave, 0x1C13, 0x02, 0x1a01);     // en dithering

    el2535_writeu8  (slave, 0x8000, 0x03, 0);     // en dithering
    el2535_writeu8  (slave, 0x8000, 0x04, 0);     // invert polarity
    el2535_writeu8  (slave, 0x8000, 0x05, 1);     // watchdog

    el2535_write16  (slave, 0x8000, 0x0B, 0);     // offset
    el2535_write32  (slave, 0x8000, 0x0C, 65536); // gain

    el2535_write16  (slave, 0x8000, 0x0D, 0);     // output in case of error
    el2535_writeu16 (slave, 0x8000, 0x0E, 65535); // output ramp

    el2535_writeu8  (slave, 0x8000, 0x10, 60);    // current limit %
    el2535_writeu16 (slave, 0x8000, 0x12, 350);   // Kp
    el2535_writeu16 (slave, 0x8000, 0x13, 6);     // Ki
    el2535_writeu16 (slave, 0x8000, 0x14, 50);    // Kd

    el2535_writeu16 (slave, 0x8000, 0x1E, 100);   // dither freq
    el2535_writeu8  (slave, 0x8000, 0x1F, 10);    // dither amp

    el2535_writeu8  (slave, 0x8000, 0x21, 0);    // info data
    el2535_writeu8  (slave, 0x8000, 0x22, 1);    // info data


    el2535_writeu8  (slave, 0x8010, 0x03, 0);     // en dithering
    el2535_writeu8  (slave, 0x8010, 0x04, 0);     // invert polarity
    el2535_writeu8  (slave, 0x8010, 0x05, 0);     // watchdog

    el2535_write16  (slave, 0x8010, 0x0B, 0);     // offset
    el2535_write32  (slave, 0x8010, 0x0C, 65536); // gain

    el2535_write16  (slave, 0x8010, 0x0D, 0);     // output in case of error
    el2535_writeu16 (slave, 0x8010, 0x0E, 65535); // output ramp

    el2535_writeu8  (slave, 0x8010, 0x10, 90);    // current limit %
    el2535_writeu16 (slave, 0x8010, 0x12, 350);   // Kp
    el2535_writeu16 (slave, 0x8010, 0x13, 6);     // Ki
    el2535_writeu16 (slave, 0x8010, 0x14, 50);    // Kd

    el2535_writeu16 (slave, 0x8010, 0x1E, 100);   // dither freq
    el2535_writeu8  (slave, 0x8010, 0x1F, 10);    // dither amp

    el2535_writeu8  (slave, 0x8000, 0x21, 0);    // info data
    el2535_writeu8  (slave, 0x8000, 0x22, 1);    // info data
    return 0;
}

typedef struct PACKED
{
    bool en_dithering     :1;
    int64_t               :4;
    bool en_cc            :1;
    bool reset            :1;
    int64_t               :9;
    int16_t pwm_out       :16;
    bool en_dithering2    :1;
    int64_t               :4;
    bool en_cc2           :1;
    bool reset2           :1;
    int64_t               :9;
    int16_t pwm_out2      :16;
} out_EL2535t;


out_EL2535t *out_el2535_1;

void set_output_int16 (uint16 slave_no, uint8 module_index, uint16 value)
{
    uint8 *data_ptr;

    data_ptr = ec_slave[slave_no].outputs;
    /* Move pointer to correct module index*/
    data_ptr += module_index * 2;
    /* Read value byte by byte since all targets can't handle misaligned
  addresses
     */
    *data_ptr++ = (value >> 0) & 0xFF;
    *data_ptr++ = (value >> 8) & 0xFF;
}

void set_output_int8 (uint16 slave_no, uint8 module_index, uint8 value)
{
    uint8 *data_ptr;

    data_ptr = ec_slave[slave_no].outputs;
    /* Move pointer to correct module index*/
    data_ptr += module_index * 2;
    /* Read value byte by byte since all targets can't handle misaligned
  addresses
     */
    *data_ptr++ = (value >> 0) & 0xFF;
    *data_ptr++ = (value >> 8) & 0xFF;
}

void simpletest(char *ifname)
{
    int i, chk;
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */


        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            if (ec_slavecount > 0)
            {
                ec_slavet * slave = &ec_slave[2];

                // el2535 drive
                if(slave->eep_man == 2 && slave->eep_id == 0x09e73052)
                {
                    printf ("Found el2535 drive\n");
                    slave->PO2SOconfig = el2535_setup;
                }
            }

            ec_slave[10].CoEdetails &= ~ECT_COEDET_SDOCA;
            ec_slave[11].CoEdetails &= ~ECT_COEDET_SDOCA;
            ec_slave[2].CoEdetails &= ~ECT_COEDET_SDOCA;


            ec_config_map(&IOmap);
            ec_configdc();


            out_el2535_1 = (out_EL2535t*) ec_slave[2].outputs;
            out_el2535_1->en_dithering = 1;
            out_el2535_1->en_cc = 1;
            out_el2535_1->en_dithering2 = 0;
            out_el2535_1->en_cc2 = 0;

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("Request operational state for all slaves\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                while(true)
                {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(5000);
                }

            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
}
