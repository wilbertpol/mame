// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NUPI disk formatter.

    Per the NUPI's 68000 firmware (see nupi.cpp), each SCSI target on the
    formatter bus is a formatter card, not a drive: the firmware selects a
    target by SCSI ID and then addresses one of up to two attached drives
    via the LUN (sent both in the IDENTIFY message and, redundantly, in
    CDB byte 1 bits 7:5, per SCSI-1 convention). This device models one
    formatter and always exposes both LUNs, so a real 2-drive formatter
    can be wired up by simply mounting a CHD in the second image slot.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_FORMATTER_H
#define MAME_TI_EXPLORER_FORMATTER_H

#pragma once

#include "machine/nscsi_hle.h"
#include "imagedev/harddriv.h"

class explorer_formatter_device : public nscsi_full_device
{
public:
	explorer_formatter_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// See nscsi_harddisk_device::set_seek_timing() (bus/nscsi/hd.h) for the
	// parameter meanings; applies identically to both LUNs.
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

DECLARE_DEVICE_TYPE(NUPI_FORMATTER, explorer_formatter_device)

#endif // MAME_TI_EXPLORER_FORMATTER_H
