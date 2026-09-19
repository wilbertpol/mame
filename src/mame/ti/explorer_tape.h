// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NUPI cartridge tape drive.

    A stock SCSI tape drive, except that it transfers at a finite rate.
    The NUPI's FIFO-to-NuBus DMA drain is real-timed, so a target that
    hands over a whole data phase inside a single step leaves it with
    nothing to move - see scsi_data_byte_period() in the source.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_TAPE_H
#define MAME_TI_EXPLORER_TAPE_H

#pragma once

#include "bus/nscsi/tape.h"

class explorer_tape_device : public nscsi_tape_device
{
public:
	explorer_tape_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	virtual attotime scsi_data_byte_period() override;
};

DECLARE_DEVICE_TYPE(NUPI_TAPE, explorer_tape_device)

#endif // MAME_TI_EXPLORER_TAPE_H
