// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NUPI mass storage unit.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_MSU_H
#define MAME_TI_EXPLORER_MSU_H

#pragma once

#include "machine/nscsi_hle.h"
#include "imagedev/harddriv.h"

class explorer_msu_device : public nscsi_full_device
{
public:
	explorer_msu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void set_seek_timing(uint32_t track_us, uint32_t average_us, uint32_t full_us, uint32_t rpm, uint8_t interleave);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void scsi_command() override;
	virtual uint8_t scsi_get_data(int id, int pos) override;
	virtual void scsi_put_data(int id, int pos, uint8_t data) override;
	virtual attotime scsi_data_command_delay() override;
	virtual attotime scsi_data_byte_period() override;

private:
	static constexpr unsigned LUN_COUNT = 2;

	struct lun_state
	{
		uint8_t block[512];
		int lba = 0;
		int cur_lba = -1;
		int blocks = 0;
		int bytes_per_sector = 0;
		std::vector<u8> inquiry_data;
		int last_cylinder = -1;
	};

	attotime seek_time(lun_state &l, uint32_t lba);

	required_device_array<harddisk_image_device, LUN_COUNT> m_image;
	lun_state m_lun[LUN_COUNT];
	int m_active_lun;
	std::string m_default_model_name;

	bool     m_seek_model = false;
	uint8_t  m_interleave = 1;
	uint32_t m_rpm = 3600;
	uint32_t m_seek_track_us = 0;
	uint32_t m_seek_range_us = 0;
	double   m_seek_exp = 1.0;
};

DECLARE_DEVICE_TYPE(EXPLORER_MSU, explorer_msu_device)

#endif // MAME_TI_EXPLORER_MSU_H
