
#include <string.h>
#include "qmc6309.h"

static qmc6309_data_t p_mag;
//static const unsigned char mag_slave[] = {QMC6309_IIC_ADDR};
static const unsigned char mag_slave[] = {QMC6309H_IIC_ADDR};

int qmc6309_read_block(unsigned char addr, unsigned char *data, unsigned short len)
{
	int ret = QMC6309_FAIL;
	int retry = 0;

	while((ret!=QMC6309_OK) && (retry++ < 5))
	{
		ret = bsp_read_reg(p_mag.slave_addr, addr, data, len);
	}

	return ret;
}

int qmc6309_write_reg(unsigned char addr, unsigned char data)
{
	int ret = QMC6309_FAIL;
	int retry = 0;

	while((ret!=QMC6309_OK) && (retry++ < 5))
	{
		ret = bsp_write_reg(p_mag.slave_addr, addr, data);
	}

	return ret;
}

void qmc6309_delay(unsigned int ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}


void qmc6309_dump_reg(void)
{
	unsigned char i2c_0_2 = 0;
	unsigned char version_id = 0;
	unsigned char wafer_id = 0;
	unsigned short d_id = 0;	
	unsigned char g_reg_tbl[16];
	
	qst_logi("\r\n********qst reg dump(hex)********\r\n");
	qst_logi("     |  0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	qst_logi("______________________________________________________\r\n");
	qmc6309_read_block(0x00, &g_reg_tbl[0], 16);
	QMC6309_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 0,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);

	qmc6309_read_block(0x10, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6309_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 1,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);
	version_id = g_reg_tbl[0x02];

	qmc6309_read_block(0x20, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6309_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 2,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);
	qmc6309_read_block(0x30, &g_reg_tbl[0], 16);
//	qmc6309_delay(1);
	QMC6309_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n", 3,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7],
																		g_reg_tbl[8],g_reg_tbl[9],g_reg_tbl[10],g_reg_tbl[11],
																		g_reg_tbl[12],g_reg_tbl[13],g_reg_tbl[14],g_reg_tbl[15]);	
	wafer_id = g_reg_tbl[0x07]&0x1f;
	d_id = (unsigned short)((g_reg_tbl[0x09]<<8)|g_reg_tbl[0x08]);
	i2c_0_2 = ((g_reg_tbl[0x07]&0x20)>>3)|(g_reg_tbl[0x0b]&0x03);

	qmc6309_read_block(0x40, &g_reg_tbl[0], 8);
//	qmc6309_delay(1);
	QMC6309_LOG("0x%xx | %02x %02x %02x %02x %02x %02x %02x %02x \r\n", 4,
																		g_reg_tbl[0],g_reg_tbl[1],g_reg_tbl[2],g_reg_tbl[3],
																		g_reg_tbl[4],g_reg_tbl[5],g_reg_tbl[6],g_reg_tbl[7]);

	QMC6309_LOG("Version-ID:[0x%02x] Wafer-ID:[0x%02x] Di-ID:[0x%04x] I2C[bit0-2]:[%x]\r\n", version_id, wafer_id, d_id, i2c_0_2);
	QMC6309_LOG("********qst reg dump done********\r\n\r\n\r\n");
}

//void qmc6309_get_chip_info(unsigned int *info)
//{
//	unsigned char verid = 0;
//	unsigned char ctrl_value[4];
	
//	qmc6309_read_block(0x37, ctrl_value, 3);
//	qmc6309_read_block(0x12, &verid, 1);
//	info[0] = (unsigned int)(((unsigned int)0x90<<24)|((unsigned int)ctrl_value[0]<<16)|((unsigned int)ctrl_value[2]<<8)|((unsigned int)ctrl_value[1]));
//	info[1] = verid;
//}

int qmc6309_get_chipid(void)
{
	int ret = QMC6309_FAIL;
	int retry=0;
	unsigned char chip_id = 0x00;

	retry = 0;
	while((chip_id != QMC6309_CHIP_ID) && (retry++<5))
	{
		ret = qmc6309_read_block(QMC6309_CHIP_ID_REG, &chip_id, 1);
		if(ret == QMC6309_OK)
		{
			break;
		}
	}
	//if((chip_id == QMC6309_CHIP_ID)||(chip_id == 0x91)||(chip_id == 0xa5))
	if(chip_id == QMC6309_CHIP_ID)
	{
		QMC6309_LOG("qmc6309_get_chipid-ok slave:0x%x chipid = 0x%x\r\n", p_mag.slave_addr, chip_id);
		return 1;
	}
	else
	{
		QMC6309_LOG("qmc6309_get_chipid-fail slave:0x%x chip_id = 0x%x\r\n", p_mag.slave_addr, chip_id);
		return 0;
	}
}

void qmc6309_set_range(unsigned char range)
{
	QMC6309_LOG("qmc6309_set_range 0x%x\r\n", range);
	p_mag.ctrl2.bit.range = range;

	switch(p_mag.ctrl2.bit.range)
	{
		case QMC6309_RNG_32G:
			p_mag.ssvt = 1000;
			break;
		case QMC6309_RNG_16G:
			p_mag.ssvt = 2000;
			break;
		case QMC6309_RNG_8G:
			p_mag.ssvt = 4000;
			break;
		default:
			p_mag.ssvt = 1000;
			break;
	}
}

int qmc6309_enable(void)
{
	int ret = 0;

	QMC6309_LOG("qmc6309_enable!\r\n");
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, p_mag.ctrl2.value);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay(1);
	ret = qmc6309_write_reg(QMC6309_CTL_REG_ONE, p_mag.ctrl1.value);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay(1);

	return ret;
}

int qmc6309_disable(void)
{
	int ret = 0;	
	qmc6309_ctrlreg1	ctrl1;

	QMC6309_LOG("qmc6309_disable!\r\n");
	ctrl1.value = p_mag.ctrl1.value;
	ctrl1.bit.mode = QMC6309_MODE_SUSPEND;
	ret = qmc6309_write_reg(QMC6309_CTL_REG_ONE, ctrl1.value);
	QMC6309_CHECK_ERR(ret);

	return ret;
}

void qmc6309_enable_ibi(qmc6309_fifo_ibi flag)
{
	int ret = QMC6309_FAIL;
	unsigned char ibi_value = 0x00;

	QMC6309_LOG("qmc6309_enable_ibi 0x%x\r\n", flag);
	ibi_value = (unsigned char)flag;
	ret = qmc6309_write_reg(QMC6309_CTL_IBI, ibi_value);

	QMC6309_CHECK_ERR(ret);
}

void qmc6309_init_para(unsigned char mode, unsigned char odr)
{
	p_mag.slave_addr = mag_slave[0];

	p_mag.ctrl1.bit.mode = mode;	// QMC6309_MODE_HPFM; QMC6309_MODE_NORMAL
	if((odr == QMC6309_ODR_200HZ) || (mode == QMC6309_MODE_HPFM))
	{
		p_mag.ctrl1.bit.osr1 = QMC6309_OSR1_8;
		p_mag.ctrl1.bit.osr2 = QMC6309_OSR2_4;
	}
	else
	{
		p_mag.ctrl1.bit.osr1 = QMC6309_OSR1_4;
		p_mag.ctrl1.bit.osr2 = QMC6309_OSR2_2;
	}

	if(p_mag.chip_type == TYPE_QMC6309)	
		p_mag.ctrl1.bit.zdbl_enb = QMC6309_ZDBL_ENB_OFF;
	else
		p_mag.ctrl1.bit.zdbl_enb = QMC6309H_ZDBL_ENB_OFF;

	p_mag.ctrl2.bit.set_rst = QMC6309_SET_RESET_ON;		// QMC6309_SET_ON, QMC6309_SET_RESET_ON	QMC6309_SET_RESET_OFF
	p_mag.ctrl2.bit.range = QMC6309_RNG_32G;
	p_mag.ctrl2.bit.odr = odr;
	p_mag.ctrl2.bit.soft_rst = 0;

	qmc6309_set_range(p_mag.ctrl2.bit.range);
#if defined(QMC6309_MODE_SWITCH)
	p_mag.set_ctl.mode = 0;
	p_mag.set_ctl.count = 0;
#endif
}


void qmc6309_reload_otp(void)
{
	int ret = 0;
	unsigned char status = 0;
	int retry = 0;
	int count = 0;

	QMC6309_LOG("qmc6309_reload_otp\r\n");
	while(retry++ < 20)
	{
		ret = qmc6309_write_reg(0x28, 0x02);
		qmc6309_delay(2);
		if(ret != QMC6309_OK)
		{
			QMC6309_LOG("write 0x28 = 0x02 fail!\r\n");
		}
		qmc6309_delay(2);
		count = 0;
		while(count++<100)
		{
			qmc6309_delay(1);
			status = 0;
			ret = qmc6309_read_block(QMC6309_STATUS_REG, &status, 1);
			if((ret==QMC6309_OK)&&(status & 0x10))
			{
				QMC6309_LOG("qmc6309_reload_otp done slave=0x%x status=0x%x\r\n", p_mag.slave_addr, status);
				//qmc6309_dump_reg();				
				return;
			}
		}
	}
	
	QMC6309_LOG("qmc6309_reload_otp fail\r\n");
}


void qmc6309_check_otp(void)
{
	int ret = QMC6309_FAIL;
	int retry = 0;
	unsigned char status = 0x00;
	int count=10;

	while(count > 0)
	{
		count--;
		retry = 0;
		while(retry++<5)
		{
			ret = qmc6309_read_block(QMC6309_STATUS_REG, &status, 1);
			QMC6309_CHECK_ERR(ret);
			QMC6309_LOG("qmc6309 status 0x%x\r\n", status);
			if(status & 0x10)
			{
				QMC6309_LOG("qmc6309 NVM load done!\r\n");
				return;
			}
			qmc6309_delay(1);
		}

		if(!(status & 0x10))
		{
			qmc6309_reload_otp();
		}
		else
		{
			return;
		}
	}
}


void qmc6309_soft_reset(void)
{
	int ret = QMC6309_FAIL;
	int retry = 0;
	unsigned char status = 0x00;

	QMC6309_LOG("qmc6309_soft_reset!\r\n");
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, 0x80);
	QMC6309_CHECK_ERR(ret);
	ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, 0x00);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay(5);

	while(retry++<5)
	{
		ret = qmc6309_read_block(QMC6309_STATUS_REG, &status, 1);
		QMC6309_CHECK_ERR(ret);
		QMC6309_LOG("qmc6309 status 0x%x\r\n", status);
		if((status & 0x10)&&(status & 0x08))
		{
			QMC6309_LOG("qmc6309 NVM load done!\r\n");
			break;
		}
		qmc6309_delay(1);
	}
	if(p_mag.chip_type == TYPE_QMC6309H)
	{
		ret = qmc6309_read_block(0x40, &status, 1);
		QMC6309_LOG("read 0x40=0x%02x ret=%d\r\n", status, ret);
#if defined(QMC6309H_0X40_CFG)
		status = (status&0xe0)|0x0d;	// 170mv
		//status = (status&0xe0)|0x0c;
		//status = status|0x80;					// set vddio=vdd disable 1.2v io auto detect
		ret = qmc6309_write_reg(0x40, status);
		QMC6309_LOG("write 0x40=0x%02x ret=%d\r\n", status, ret);
		qmc6309_delay(1);
#endif
	}
}


int qmc6309_read_mag_raw(short raw[3])
{
	int res = QMC6309_FAIL;
	unsigned char mag_data[6];
	int t1 = 0;
	unsigned char rdy = 0;

	/* Check status register for data availability */
	res = qmc6309_read_block(QMC6309_STATUS_REG, &rdy, 1);
	while(!(rdy & (QMC6309_STATUS_DRDY|QMC6309_STATUS_OVFL)) & (t1++ < 5))
	{
		res = qmc6309_read_block(QMC6309_STATUS_REG, &rdy, 1);
		QMC6309_CHECK_ERR(res);
		qmc6309_delay(1);
	}
	if((res == QMC6309_FAIL)||(!(rdy & QMC6309_STATUS_DRDY)))
	{
		raw[0] = p_mag.last_data[0];
		raw[1] = p_mag.last_data[1];
		raw[2] = p_mag.last_data[2];
		QMC6309_LOG("qmc6309_read_mag_raw read drdy fail! res=%d rdy=0x%x\r\n",res,rdy);
		res = QMC6309_OK;	// QMC6309_FAIL;
	}
	else if(rdy & QMC6309_STATUS_OVFL)
	{
		raw[0] = 32767;
		raw[1] = 32767;
		raw[2] = 32767;
		p_mag.fail_num = 0;
		return QMC6309_OK;
	}
	else
	{
		mag_data[0] = QMC6309_DATA_OUT_X_LSB_REG;
		res = qmc6309_read_block(QMC6309_DATA_OUT_X_LSB_REG, mag_data, 6);
		if(res == QMC6309_FAIL)
	  	{
			QMC6309_LOG("qmc6309_read_mag_raw read data fail! res=%d\r\n",res);
			raw[0] = p_mag.last_data[0];
			raw[1] = p_mag.last_data[1];
			raw[2] = p_mag.last_data[2];
			res = QMC6309_OK;	// QMC6309_FAIL;
		}
		else
		{
			raw[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
			raw[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
			raw[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);
		}
	}

	if((p_mag.last_data[0]==raw[0])&&(p_mag.last_data[1]==raw[1])&&(p_mag.last_data[2]==raw[2]))
	{
		p_mag.fail_num++;
	}
	else
	{
		p_mag.fail_num = 0;
	}

	if(p_mag.fail_num > 10)
	{
#if defined(QMC6309_RECOVER)
		//qmc6309_recover();
		qmc6309_soft_reset();
		qmc6309_enable();
#endif
		p_mag.fail_num = 0;
	}
	
	p_mag.last_data[0] = raw[0];
	p_mag.last_data[1] = raw[1];
	p_mag.last_data[2] = raw[2];	
#if defined(QMC6309_MODE_SWITCH)
	qmc6309_setrst_auto_mode(raw);
#endif

	return res;
}


int qmc6309_read_mag_xyz(float uT[3])
{
	int res = QMC6309_FAIL;
	short raw[3];

	res = qmc6309_read_mag_raw(raw);
	if(res == QMC6309_OK)
	{
		uT[0] = (float)((float)raw[0] / ((float)p_mag.ssvt/100.f));		// ut
		uT[1] = (float)((float)raw[1] / ((float)p_mag.ssvt/100.f));		// ut
		uT[2] = (float)((float)raw[2] / ((float)p_mag.ssvt/100.f));		// ut
	}
	else
	{
		uT[0] = uT[1]= uT[2] = 0.0f;
	}

	if(p_mag.ctrl1.bit.mode == QMC6309_MODE_SINGLE)
	{
		res = qmc6309_write_reg(QMC6309_CTL_REG_ONE, p_mag.ctrl1.value);
		QMC6309_CHECK_ERR(res);
	}

	return res;
}


int qmc6309_self_test(void)
{
	int selftest_result = 0;
	int selftest_retry = 0;
	signed char  st_data[3];
	unsigned char abs_data[3];
	unsigned char rdy = 0x00;
	int t1 = 0;
	int ret = QMC6309_FAIL;

	while((selftest_result == 0)&&(selftest_retry<3))
	{
		selftest_retry++;
		qmc6309_write_reg(QMC6309_CTL_REG_ONE, 0x00);
		qmc6309_delay(2);
		qmc6309_write_reg(QMC6309_CTL_REG_TWO, 0x00);
		qmc6309_delay(2);
		qmc6309_write_reg(QMC6309_CTL_REG_ONE, 0x03);
		qmc6309_delay(20);
		qmc6309_write_reg(0x0e, 0x80);
		//qmc6309_delay(150);	// old 150ms
		rdy = 0x00;
		t1 = 0;
		while(!(rdy & 0x04))
		{
			qmc6309_delay(5);
			ret = qmc6309_read_block(QMC6309_STATUS_REG, &rdy, 1);

			if(t1++ > 50)
			{
				break;
			}
		}		

		if(rdy & 0x04)
		{
			ret = qmc6309_read_block(QMC6309_DATA_OUT_ST_X, (unsigned char*)st_data, 3);
			if(ret == QMC6309_FAIL)
				continue;
		}
		else
		{
			QMC6309_LOG("qmc6309 selftest drdy fail!\r\n");
			continue;
		}

		abs_data[0] = QMC6309_ABS(st_data[0]);
		abs_data[1] = QMC6309_ABS(st_data[1]);
		abs_data[2] = QMC6309_ABS(st_data[2]);

		if(	((abs_data[0] < QMC6309_SELFTEST_MAX_X) && (abs_data[0] > QMC6309_SELFTEST_MIN_X))
			&& ((abs_data[1] < QMC6309_SELFTEST_MAX_Y) && (abs_data[1] > QMC6309_SELFTEST_MIN_Y))
			&& ((abs_data[2] < QMC6309_SELFTEST_MAX_Z) && (abs_data[2] > QMC6309_SELFTEST_MIN_Z)) )
	    {
			QMC6309_LOG("qmc6309 selftest OK! status[0x%x]data[%d	%d	%d]\r\n",rdy,st_data[0],st_data[1],st_data[2]);
	        selftest_result = 1;
	    }
		else
		{
			QMC6309_LOG("qmc6309 selftest fail! status[0x%x]data[%d	%d	%d]\r\n",rdy,st_data[0],st_data[1],st_data[2]);
			selftest_result = 0;
		}
	}

	return selftest_result;
}


#if defined(QMC6309_MODE_SWITCH)
void qmc6309_setrst_auto_mode(short hw_d[3])
{
	int ret = QMC6309_FAIL;

	if(p_mag.set_ctl.mode == 0)
	{// set reset on
		if((QMC6309_ABS(hw_d[0]) > 8000)||(QMC6309_ABS(hw_d[1]) > 8000))
		{
			p_mag.set_ctl.count++;
			if(p_mag.set_ctl.count >= 10)
			{
				p_mag.set_ctl.mode = 1;
				p_mag.set_ctl.count = 0;

				p_mag.ctrl2.bit.set_rst = QMC6309_SET_ON;
				ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, p_mag.ctrl2.value);
				QMC6309_CHECK_ERR(ret);
				qmc6309_delay(1);
			}
		}
		else
		{
			p_mag.set_ctl.count = 0;
		}
	}
	else
	{// set only
		int force_switch = 0;
		p_mag.set_ctl.count++;
		if(p_mag.set_ctl.count >= 100)
		{
			p_mag.set_ctl.count = 0;
			force_switch = 1;
		}
		if(((QMC6309_ABS(hw_d[0]) < 6000)&&(QMC6309_ABS(hw_d[1]) < 6000)) || force_switch)
		{
			p_mag.set_ctl.mode = 0;
			p_mag.set_ctl.count = 0;

			p_mag.ctrl2.bit.set_rst = QMC6309_SET_RESET_ON;
			ret = qmc6309_write_reg(QMC6309_CTL_REG_TWO, p_mag.ctrl2.value);
			QMC6309_CHECK_ERR(ret);
			qmc6309_delay(1);
		}
	}

}
#endif

void qmc6309_fifo_config(qmc6309_fifo_mode mode, qmc6309_fifo_ch ch, int wmk)
{
	unsigned char fifo_reg = 0x00;
	int ret = QMC6309_OK;

	fifo_reg = (mode|ch|(wmk<<3));
	p_mag.fifo_ctrl = fifo_reg;
	ret = qmc6309_write_reg(QMC6309_FIFO_REG_CTRL, fifo_reg);
	QMC6309_CHECK_ERR(ret);
	qmc6309_delay(1);
	p_mag.fifo_frame_len = 0;
	if(ch & QMC6309_FIFO_CH_X)
	{
		p_mag.fifo_frame_len+=2;
	}	
	if(ch & QMC6309_FIFO_CH_Y)
	{
		p_mag.fifo_frame_len+=2;
	}
	if(ch & QMC6309_FIFO_CH_Z)
	{
		p_mag.fifo_frame_len+=2;
	}
}

int qmc6309_fifo_read(unsigned char *f_data)
{
	unsigned char fifo_status = 0;
	unsigned char fifo_level = 0;
	int ret = QMC6309_FAIL;

	ret = qmc6309_read_block(QMC6309_FIFO_REG_STATUS, &fifo_status, 1);
	QMC6309_CHECK_ERR(ret);
	fifo_level = (fifo_status >> 4);
	if(fifo_level)
	{
		ret = qmc6309_read_block(QMC6309_FIFO_REG_DATA, (unsigned char*)f_data, p_mag.fifo_frame_len*fifo_level);
		//for(int index=0; index<fifo_level; index++)
		//{
		//	qmc6309_read_block(QMC6309_FIFO_REG_DATA, &f_data[index*6], p_mag.fifo_frame_len);
		//}
		//ret = qmc6309_write_reg(QMC6309_FIFO_REG_CTRL, p_mag.fifo_ctrl);
		//QMC6309_LOG("status:0x%x	fifo-level:%d\r\n", fifo_status, fifo_level);
		QMC6309_CHECK_ERR(ret);		
		p_mag.fail_num = 0;
	}
	else
	{
		p_mag.fail_num++;
		if(p_mag.fail_num >= 2)
		{
			qmc6309_soft_reset();
			qmc6309_write_reg(QMC6309_FIFO_REG_CTRL, p_mag.fifo_ctrl);
			qmc6309_enable();
			p_mag.fail_num = 0;
		}
	}

	return (int)fifo_level;
}

#if defined(QMC6309_RECOVER)
int qmc6309_recover(void)
{
	int ret = QMC6309_FAIL;
	unsigned char slave_loop[]={0x7c,0x0c,0x1c,0x2c,0x3c,0x4c,0x5c,0x6c};

	for(int i=0; i<sizeof(slave_loop)/sizeof(slave_loop[0]); i++)
	{
		p_mag.chip_type = TYPE_UNKNOW;
		p_mag.slave_addr = slave_loop[i];
		ret = qmc6309_get_chipid();	// read id again
		if(ret) 	// read id OK
		{
			QMC6309_LOG("qmc6309_recover slave=0x%02x read id OK\r\n", p_mag.slave_addr);
			qmc6309_soft_reset();	// softreset reload OTP
			//qmc6309_reload_otp();	// reload otp
			p_mag.chip_type = TYPE_UNKNOW;
			for(i=0; i<sizeof(mag_slave)/sizeof(mag_slave[0]); i++)
			{
				p_mag.slave_addr = mag_slave[i];
				ret = qmc6309_get_chipid();
				if(ret)
				{
					if(p_mag.slave_addr == QMC6309_IIC_ADDR)
					{
						p_mag.chip_type = TYPE_QMC6309;
					}
					else if(p_mag.slave_addr == QMC6309H_IIC_ADDR)
					{
						p_mag.chip_type = TYPE_QMC6309H;
					}
					else
					{
						p_mag.chip_type = TYPE_UNKNOW;
					}
					break;
				}
			}

			break;
		}
	}
 
	QMC6309_LOG("qmc6309_recover %s mag_type=%d\r\n", ret?"OK":"FAIL", p_mag.chip_type);
	return ret;
}
#endif

int qmc6309_init(void)
{
	int ret = 0;
	int i = 0;

	memset(&p_mag, 0, sizeof(p_mag));
	p_mag.chip_type = TYPE_UNKNOW;
	for(i=0; i<sizeof(mag_slave)/sizeof(mag_slave[0]); i++)
	{
		p_mag.slave_addr = mag_slave[i];
		ret = qmc6309_get_chipid();
		if(ret)
		{
			if(p_mag.slave_addr == QMC6309_IIC_ADDR)
			{
				p_mag.chip_type = TYPE_QMC6309;
			}
			else if(p_mag.slave_addr == QMC6309H_IIC_ADDR)
			{
				p_mag.chip_type = TYPE_QMC6309H;
			}
			else
			{
				p_mag.chip_type = TYPE_UNKNOW;
			}
			break;
		}
	}

#if defined(QMC6309_RECOVER)
	if(p_mag.chip_type == TYPE_UNKNOW)
	{
		qmc6309_recover();
	}
#endif
	if(p_mag.chip_type != TYPE_UNKNOW)
	{
		qmc6309_soft_reset();		
		qmc6309_init_para(QMC6309_MODE_HPFM, QMC6309_ODR_HPFM);
		QMC6309_CHECK_ERR(ret);
		ret = qmc6309_enable();
		QMC6309_CHECK_ERR(ret);
		//qmc6309_dump_reg();

		return 1;
	}
	else
	{
		return 0;
	}
}


