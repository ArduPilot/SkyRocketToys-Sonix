/**
 * @file
 * MAVLink Remote Logging
 * @author pbarker
 */

#include <stdlib.h>
#include <string.h>
#include <libmid_fatfs/ff.h>
#include <sys_clock.h>
#include "dev_console.h"

#include "mavlink_remote_log.h"

static FIL log_fh;
static const uint8_t start_interval = 1; // seconds
static long long last_start_attempt = 0; // time (seconds)
static const uint8_t stop_interval = 1; // seconds
static long long last_stop_attempt = 0; // time (seconds)
static uint32_t last_message_received_ms;
static bool need_sync;

static long long last_lastlog_filepath_warning = 0;
static long long last_lastlog_logfh_warning = 0;

static bool setup_done = false;
static bool enabled = true;
static char log_filepath[300];
static uint32_t ack_counter;

static void mavlink_remote_log_setup()
{
    if (setup_done) {
	return;
    }
    setup_done = true;
}

static void ensure_log_basedir_exists()
{
    // ensure the logging directory exists:
    if (!exists(REMOTE_LOG_BASEDIR)) {
	if (!mkdir(REMOTE_LOG_BASEDIR)) {
            static long long last_report;
            long long now = get_sys_seconds_boot();
            if (now - last_report > 10) {
                console_printf("rl: failed to mkdir " REMOTE_LOG_BASEDIR "\n");
                last_report = now;
            }
	}
    }
}

/*
 * the number of the last log created, from LASTLOG.TXT; 0 if unknown
 */
static uint16_t read_lastlog_number(void)
{
    const char *lastlog_filepath = REMOTE_LOG_BASEDIR "/LASTLOG.TXT";
    uint16_t number = 0;
    FIL lastlog_fptr;
    if (f_open(&lastlog_fptr, lastlog_filepath, FA_READ) == FR_OK) {
	char buf[16];
        memset(buf, '\0', sizeof(buf));
	unsigned int read_count;
	if (f_read(&lastlog_fptr, buf, sizeof(buf)-1, &read_count) == FR_OK) {
	    number = atoi(buf);
	}
	f_close(&lastlog_fptr);
    }
    return number;
}

/*
 * the highest log number in use in the log directory, or min if
 * that is higher.  Reads every directory entry, each a separate
 * call into FatFs's task, so only for when LASTLOG.TXT is well
 * behind
 */
static uint16_t highest_log_number(uint16_t min)
{
    uint16_t highest = min;
    DIR dir;
    if (f_opendir(&dir, REMOTE_LOG_BASEDIR) != FR_OK) {
        return highest;
    }
    FILINFO fno;
    memset(&fno, 0, sizeof(fno));
    // names are 8.3, so the short name is enough
    fno.lfname = NULL;
    fno.lfsize = 0;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
        if (fno.fattrib & AM_DIR) {
            continue;
        }
        // NNNNNN.BIN
        const char *name = fno.fname;
        unsigned number = 0;
        unsigned i;
        for (i = 0; i < 6 && name[i] >= '0' && name[i] <= '9'; i++) {
            number = number * 10 + (name[i] - '0');
        }
        // the extension in either case: FatFs gives ".bin" for a
        // short name that a PC, or FatFs itself, marked lower case
        const char *ext = &name[6];
        if (i == 6 && ext[0] == '.' &&
            (ext[1] & ~0x20) == 'B' && (ext[2] & ~0x20) == 'I' &&
            (ext[3] & ~0x20) == 'N' && ext[4] == '\0' &&
            number <= 0xFFFF && number > highest) {
            highest = number;
        }
    }
    f_closedir(&dir);
    return highest;
}

/*
 * record the number of the last log created in LASTLOG.TXT.
 *
 * It is overwritten in place and truncated, not opened with
 * FA_CREATE_ALWAYS: truncating on open frees the file's cluster and
 * makes FatFs search for the next free cluster from there, and as
 * LASTLOG.TXT sits low on the card, the new log's first write then
 * read the FAT past every used cluster after it, which took over 3
 * seconds.  Nor is FA_OPEN_ALWAYS used, as this FatFs refuses that
 * for a file which exists.
 */
static void write_lastlog_number(uint16_t number)
{
    const char *lastlog_filepath = REMOTE_LOG_BASEDIR "/LASTLOG.TXT";
    FIL lastlog_fptr;
    FRESULT res = f_open(&lastlog_fptr, lastlog_filepath, FA_OPEN_EXISTING|FA_WRITE);
    if (res == FR_NO_FILE) {
        res = f_open(&lastlog_fptr, lastlog_filepath, FA_CREATE_NEW|FA_WRITE);
    } else if (res == FR_DENIED) {
        // read-only, perhaps set on a PC; a marker that can't be
        // updated would leave every new log searching from it
        if (f_chmod(lastlog_filepath, 0, AM_RDO) == FR_OK) {
            res = f_open(&lastlog_fptr, lastlog_filepath, FA_OPEN_EXISTING|FA_WRITE);
        }
    }
    if (res != FR_OK) {
	// moan about not being able to open the file
        const long long now = get_sys_seconds_boot();
        if (last_lastlog_filepath_warning == 0 ||
            now - last_lastlog_filepath_warning > 10) {
            console_printf("rl: failed to open (%s) (%d)\n", lastlog_filepath, (int)res);
            last_lastlog_filepath_warning = now;
        }
        return;
    }
    char buf[16];
    memset(buf, '\0', sizeof(buf));
    int used_bytes = snprintf(buf, sizeof(buf), "%u", number);
    uint32_t bytes_written;
    if (f_write(&lastlog_fptr, buf, used_bytes, &bytes_written) == FR_OK) {
	if (bytes_written != used_bytes) {
	    // whinge about short write
	    console_printf("rl: short write\n");
	}
    } else {
	// whinge about failed write
	console_printf("rl: failed write\n");
    }
    // drop anything left from a longer number
    f_truncate(&lastlog_fptr);
    f_sync(&lastlog_fptr);
    f_close(&lastlog_fptr);
}

/*
 * open a file to write blocks of remote dataflash log into
 * returns -1 on failure
 */
static int log_fh_open()
{
    ensure_log_basedir_exists();

    // never overwrite a log: if LASTLOG.TXT is behind, skip any
    // numbers already in use.  Each try searches the directory, so
    // if a few don't find a free number (LASTLOG.TXT is missing or
    // well behind), go to the highest number in use
    uint16_t number = read_lastlog_number();
    FRESULT res = FR_EXIST;
    for (uint16_t tries = 0; tries < 1000 && res == FR_EXIST; tries++) {
        if (tries == 5) {
            // unless the highest is the last number: going on from
            // there would wrap back to the numbers already tried
            const uint16_t highest = highest_log_number(number);
            if (highest != 0xFFFF) {
                number = highest;
            }
        }
        number++;
        if (number == 0) {
            number = 1;
        }
        snprintf(log_filepath, sizeof(log_filepath), REMOTE_LOG_BASEDIR "/%06u.BIN", number);
        res = f_open(&log_fh, log_filepath, FA_CREATE_NEW|FA_WRITE);
    }
    if (res != FR_OK) {
        const long long now = get_sys_seconds_boot();
        if (last_lastlog_logfh_warning == 0 ||
            now - last_lastlog_logfh_warning > 10) {
            console_printf("rl: failed to open (%s) (%d)\n", log_filepath, (int)res);
            last_lastlog_logfh_warning = now;
        }
	return -1;
    }
    write_lastlog_number(number);
    console_printf("rl: new dataflash log (%s)\n", log_filepath);
    return 0;
}

/*
 * close dataflash log file
 */
static void log_fh_close()
{
    unsigned int file_size = (unsigned)f_size(&log_fh);
    f_close(&log_fh);
    log_fh.fs = 0; // OS doesn't do this if it fails to close...

    // remove any very small log files.  This can happen when we
    // timeout after receiving a critical message
    if (file_size < 42000) {
        console_printf("rl: small log (%u bytes).  Removing.\n", (unsigned)file_size);
        f_unlink(log_filepath);
    }
}

/*
 * indicate whether log is open for writing
 */
static const int log_fh_is_open()
{
    return (log_fh.fs != 0);
}


/*
 * send an acknowledgement to the server that we have received a block
 * acks are also used to start/stop logging sessions
 */
static void send_ack(const uint32_t seqno)
{
    mavlink_msg_remote_log_block_status_send(MAVLINK_COMM_FC,
					     MAVLINK_TARGET_SYSTEM_ID,
					     MAV_COMP_ID_LOG,
					     seqno,
					     1);
    ack_counter++;
}

/*
 * attempt to stop logging - but not too often
 */
static void attempt_stop()
{
    const long long now = get_sys_seconds_boot();
    if (now - last_stop_attempt < stop_interval) {
	return;
    }
    last_stop_attempt = now;
    send_ack(MAV_REMOTE_LOG_DATA_BLOCK_STOP);
}

/*
 * attempt to start logging - but not too often
 */
static void attempt_start()
{
    const long long now = get_sys_seconds_boot();

    if (now - last_start_attempt < start_interval) {
	return;
    }
    //console_printf("rl: Sending remote-log start request\n");
    last_start_attempt = now;
    send_ack(MAV_REMOTE_LOG_DATA_BLOCK_START);
}

/*
 * write received data to disk
 */
void mavlink_handle_remote_log_data_block(mavlink_remote_log_data_block_t *msg)
{
    if (msg->target_system != MAVLINK_SYSTEM_ID ||
	msg->target_component != MAVLINK_COMPONENT_ID_REMOTE_LOG) {
	/* This shouldn't ever happen. */
	return;
    }

    if (!enabled) {
	// we could attempt_stop() here.
	return;
    }

    if (!log_fh_is_open()) {
	if (msg->seqno == 0) {
	    /* first block received - start a new block */
	    if (log_fh_open() == -1) {
		return;
	    }
	} else if (msg->seqno > 20) {
	    /* assume old logging session is in progress, useless from
	       our perspective.  Try to close that one off so we can
	       start another: */
	    attempt_stop();
            return;
	} else {
            // low-numbered block received; perhaps we missed the first one...
            return;
        }
    }

    last_message_received_ms = get_time_boot_ms();

    const uint32_t offset = MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN*msg->seqno;
    if (f_lseek(&log_fh, offset) != FR_OK) {
	/* seek failed?  Close this file and try opening another one. */
	console_printf("rl: seek (%u) failed\n", offset);
	log_fh_close();
	return;
    }
    unsigned int written;
    if (f_write(&log_fh, msg->data, MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN, &written) != FR_OK) {
	/* write failure; close this file and try opening another one */
	console_printf("rl: write failed\n");
	log_fh_close();
	return;
    }

    if (written != MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
	/* short write?  Could have been interrrupted, drop block and
	   let retries fix it. */
	return;
    }

    need_sync = true;

    // ack this block
    send_ack(msg->seqno);

    // make sure we get data to disk, write at multiple of block size
    // of fs and remote logging
    if (offset % (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN * 512) == 0) {
        f_sync(&log_fh);
        need_sync = false;
    }

}

/*
  flush to disk if we haven't written anything for 1 second. This is
  to ensure we flush on disarm
 */
void mavlink_remote_log_sync(bool force)
{
    if (log_fh_is_open() && need_sync &&
        (force || get_time_boot_ms() - last_message_received_ms > 1000)) {
        f_sync(&log_fh);        
        need_sync = false;
    }
}

void mavlink_remote_log_periodic()
{
    // filesystem isn't necessarily ready at boot, so we always call
    // this (and it does nothing once things are set up)
    mavlink_remote_log_setup();

    if (enabled) {
        if (log_fh_is_open()) {
            const uint32_t now = get_time_boot_ms();
            const uint32_t delta = now - last_message_received_ms;
            if (delta > 10000) {
                console_printf("rl: no messages received in %u seconds; closing output\n", (unsigned)(delta/1000));
                log_fh_close();
            }
        } else {
            attempt_start();
        }
    } else {
        if (log_fh_is_open()) {
            log_fh_close();
        }
    }
}


/*
  command-line support
*/
void cmd_rl_status(unsigned argc, const char *argv[])
{
    console_printf("rl: ack_counter %u\n", (unsigned)ack_counter);
    if (strlen(log_filepath)) {
	console_printf("rl: log_filepath: %s\n", log_filepath);
    }
    if (!enabled) {
	console_printf("rl: Disabled\n");
	return;
    }
    if (log_fh_is_open()) {
	console_printf("rl: Output file size: %u\n", (unsigned)f_size(&log_fh));
    } else {
	console_printf("rl: Output not opened\n");
    }
}

void cmd_rl_disable(unsigned argc, const char *argv[])
{
    if (!enabled ) {
	console_printf("rl: Already disabled\n");
	return;
    }
    if (log_fh_is_open()) {
	log_fh_close();
    }
    console_printf("rl: Disabled\n");
    enabled = false;
}

void cmd_rl_enable(unsigned argc, const char *argv[])
{
    if (enabled) {
	console_printf("rl: Already enabled\n");
	return;
    }
    console_printf("rl: Enabled\n");
    enabled = true;
}

void cmd_rl_help(unsigned argc, const char *argv[])
{
    console_printf("rl: Usage: rl help|enable|disable|status\n");
}
