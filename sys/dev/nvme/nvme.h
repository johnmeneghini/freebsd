/*-
 * Copyright (C) 2012-2013 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef __NVME_H__
#define __NVME_H__

#ifdef _KERNEL
#include <sys/types.h>
#endif

#include <sys/param.h>

/* Cap nvme to 1MB transfers driver explodes with larger sizes */
#define NVME_MAX_XFER_SIZE              (MAXPHYS < (1<<20) ? MAXPHYS : (1<<20))

#define FREEBSD_NATIVE
#ifdef static_assert
#define SPDK_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define SPDK_STATIC_ASSERT(cond, msg)
#endif

#include "nvme_spec.h"

#define	NVME_PASSTHROUGH_CMD		_IOWR('n', 0, struct nvme_pt_command)
#define	NVME_RESET_CONTROLLER		_IO('n', 1)

#define	NVME_IO_TEST			_IOWR('n', 100, struct nvme_io_test)
#define	NVME_BIO_TEST			_IOWR('n', 101, struct nvme_io_test)

#define NVME_TEST_MAX_THREADS	128

struct nvme_io_test {

	enum nvme_nvm_opcode	opc;
	uint32_t		size;
	uint32_t		time;	/* in seconds */
	uint32_t		num_threads;
	uint32_t		flags;
	uint64_t		io_completed[NVME_TEST_MAX_THREADS];
};

enum nvme_io_test_flags {

	/*
	 * Specifies whether dev_refthread/dev_relthread should be
	 *  called during NVME_BIO_TEST.  Ignored for other test
	 *  types.
	 */
	NVME_TEST_FLAG_REFTHREAD =	0x1,
};

struct nvme_pt_command {

	/*
	 * cmd is used to specify a passthrough command to a controller or
	 *  namespace.
	 *
	 * The following fields from cmd may be specified by the caller:
	 *	* opc  (opcode)
	 *	* nsid (namespace id) - for admin commands only
	 *	* cdw10-cdw15
	 *
	 * Remaining fields must be set to 0 by the caller.
	 */
	struct nvme_command	cmd;

	/*
	 * cpl returns completion status for the passthrough command
	 *  specified by cmd.
	 *
	 * The following fields will be filled out by the driver, for
	 *  consumption by the caller:
	 *	* cdw0
	 *	* status (except for phase)
	 *
	 * Remaining fields will be set to 0 by the driver.
	 */
	struct nvme_completion	cpl;

	/* buf is the data buffer associated with this passthrough command. */
	void *			buf;

	/*
	 * len is the length of the data buffer associated with this
	 *  passthrough command.
	 */
	uint32_t		len;

	/*
	 * is_read = 1 if the passthrough command will read data into the
	 *  supplied buffer from the controller.
	 *
	 * is_read = 0 if the passthrough command will write data from the
	 *  supplied buffer to the controller.
	 */
	uint32_t		is_read;

	/*
	 * driver_lock is used by the driver only.  It must be set to 0
	 *  by the caller.
	 */
	struct mtx *		driver_lock;
};

#define nvme_completion_is_error(cpl)					\
	((cpl)->status.sc != 0 || (cpl)->status.sct != 0)

void	nvme_strvis(uint8_t *dst, const uint8_t *src, int dstlen, int srclen);

#ifdef _KERNEL

struct bio;

struct nvme_namespace;
struct nvme_controller;
struct nvme_consumer;

typedef void (*nvme_cb_fn_t)(void *, const struct nvme_completion *);

typedef void *(*nvme_cons_ns_fn_t)(struct nvme_namespace *, void *);
typedef void *(*nvme_cons_ctrlr_fn_t)(struct nvme_controller *);
typedef void (*nvme_cons_async_fn_t)(void *, const struct nvme_completion *,
				     uint32_t, void *, uint32_t);
typedef void (*nvme_cons_fail_fn_t)(void *);

enum nvme_namespace_flags {
	NVME_NS_DEALLOCATE_SUPPORTED	= 0x1,
	NVME_NS_FLUSH_SUPPORTED		= 0x2,
};

int	nvme_ctrlr_passthrough_cmd(struct nvme_controller *ctrlr,
				   struct nvme_pt_command *pt,
				   uint32_t nsid, int is_user_buffer,
				   int is_admin_cmd);

/* Admin functions */
void	nvme_ctrlr_cmd_set_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_feature(struct nvme_controller *ctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_log_page(struct nvme_controller *ctrlr,
				    uint8_t log_page, uint32_t nsid,
				    void *payload, uint32_t payload_size,
				    nvme_cb_fn_t cb_fn, void *cb_arg);

/* NVM I/O functions */
int	nvme_ns_cmd_write(struct nvme_namespace *ns, void *payload,
			  uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_cmd_write_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_read(struct nvme_namespace *ns, void *payload,
			 uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			 void *cb_arg);
int	nvme_ns_cmd_read_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_deallocate(struct nvme_namespace *ns, void *payload,
			       uint8_t num_ranges, nvme_cb_fn_t cb_fn,
			       void *cb_arg);
int	nvme_ns_cmd_flush(struct nvme_namespace *ns, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_dump(struct nvme_namespace *ns, void *virt, off_t offset,
		     size_t len);

/* Registration functions */
struct nvme_consumer *	nvme_register_consumer(nvme_cons_ns_fn_t    ns_fn,
					       nvme_cons_ctrlr_fn_t ctrlr_fn,
					       nvme_cons_async_fn_t async_fn,
					       nvme_cons_fail_fn_t  fail_fn);
void		nvme_unregister_consumer(struct nvme_consumer *consumer);

/* Controller helper functions */
device_t	nvme_ctrlr_get_device(struct nvme_controller *ctrlr);
const struct nvme_controller_data *
		nvme_ctrlr_get_data(struct nvme_controller *ctrlr);

/* Namespace helper functions */
uint32_t	nvme_ns_get_max_io_xfer_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_sector_size(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_num_sectors(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_flags(struct nvme_namespace *ns);
const char *	nvme_ns_get_serial_number(struct nvme_namespace *ns);
const char *	nvme_ns_get_model_number(struct nvme_namespace *ns);
const struct nvme_namespace_data *
		nvme_ns_get_data(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_stripesize(struct nvme_namespace *ns);

int	nvme_ns_bio_process(struct nvme_namespace *ns, struct bio *bp,
			    nvme_cb_fn_t cb_fn);

/* Command building helper functions -- shared with CAM */
static inline
void	nvme_ns_flush_cmd(struct nvme_command *cmd, uint16_t nsid)
{

	cmd->opc = NVME_OPC_FLUSH;
	cmd->nsid = nsid;
}

static inline
void	nvme_ns_rw_cmd(struct nvme_command *cmd, uint32_t rwcmd, uint16_t nsid,
    uint64_t lba, uint32_t count)
{
	cmd->opc = rwcmd;
	cmd->nsid = nsid;
	*(uint64_t *)&cmd->cdw10 = lba;
	cmd->cdw12 = count-1;
	cmd->cdw13 = 0;
	cmd->cdw14 = 0;
	cmd->cdw15 = 0;
}

static inline
void	nvme_ns_write_cmd(struct nvme_command *cmd, uint16_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_WRITE, nsid, lba, count);
}

static inline
void	nvme_ns_read_cmd(struct nvme_command *cmd, uint16_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_READ, nsid, lba, count);
}

static inline
void	nvme_ns_trim_cmd(struct nvme_command *cmd, uint16_t nsid,
    uint32_t num_ranges)
{
	cmd->opc = NVME_OPC_DATASET_MANAGEMENT;
	cmd->nsid = nsid;
	cmd->cdw10 = num_ranges - 1;
	cmd->cdw11 = NVME_DSM_ATTR_DEALLOCATE;
}

#endif /* _KERNEL */

#endif /* __NVME_H__ */
