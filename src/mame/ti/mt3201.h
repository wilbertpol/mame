// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    MT3201 magnetic tape drive.

    A half-inch reel-to-reel drive taking 7, 8.5 and 10 inch reels,
    phase encoded at 1600 bpi, housed in its own enclosure and attached
    to the NUPI's SCSI bus. A Cipher CacheTape under TI's model number,
    see the source. Not to be confused with the quarter-inch cartridge
    drive that goes in the mass storage unit.

    A stock SCSI tape drive, except that it transfers at a finite rate.
    The NUPI's FIFO-to-NuBus DMA drain is real-timed, so a target that
    hands over a whole data phase inside a single step leaves it with
    nothing to move - see scsi_data_byte_period() in the source.

**********************************************************************/

#ifndef MAME_TI_MT3201_H
#define MAME_TI_MT3201_H

#pragma once

#include "bus/nscsi/tape.h"

class mt3201_device : public nscsi_tape_device
{
public:
	mt3201_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	virtual attotime scsi_data_byte_period() override;
};

DECLARE_DEVICE_TYPE(MT3201, mt3201_device)

#endif // MAME_TI_MT3201_H
