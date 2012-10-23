/**
 * fs/f2fs/gc.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define GC_THREAD_NAME	"f2fs_gc_task"
#define GC_THREAD_MIN_WB_PAGES		1	/*
						 * a threshold to determine
						 * whether IO subsystem is idle
						 * or not
						 */
#define GC_THREAD_MIN_SLEEP_TIME	10000 /* milliseconds */
#define GC_THREAD_MAX_SLEEP_TIME	30000
#define GC_THREAD_NOGC_SLEEP_TIME	10000
#define LIMIT_INVALID_BLOCK	40 /* percentage over total user space */
#define LIMIT_FREE_BLOCK	40 /* percentage over invalid + free space */

/* Search max. number of dirty segments to select a victim segment */
#define MAX_VICTIM_SEARCH	20

enum {
	GC_NONE = 0,
	GC_ERROR,
	GC_OK,
	GC_NEXT,
	GC_BLOCKED,
	GC_DONE,
};

#ifdef CONFIG_F2FS_STAT_FS
struct f2fs_stat_info {
	struct f2fs_sb_info *sbi;
	struct mutex stat_list;
	int all_area_segs;
	int sit_area_segs;
	int nat_area_segs;
	int ssa_area_segs;
	int main_area_segs;
	int main_area_sections;
	int main_area_zones;
	int hit_ext, total_ext;
	int ndirty_node;
	int ndirty_dent;
	int ndirty_dirs;
	int ndirty_meta;
	int nats, sits, fnids;
	int total_count;
	int utilization;
	int bg_gc;
	unsigned int valid_count;
	unsigned int valid_node_count;
	unsigned int valid_inode_count;
	unsigned int bimodal, avg_vblocks;
	int util_free, util_valid, util_invalid;
	int rsvd_segs, overp_segs;
	int dirty_count;
	int node_pages;
	int meta_pages;
	int prefree_count;
	int call_count;
	int tot_segs;
	int node_segs;
	int data_segs;
	int free_segs;
	int free_secs;
	int tot_blks;
	int data_blks;
	int node_blks;
	int curseg[6];
	int cursec[6];
	int curzone[6];

	unsigned int segment_count[2];
	unsigned int block_count[2];
};

#define GC_STAT_I(gi)			((gi)->stat_info)

#define gc_stat_inc_call_count(gi)	((GC_STAT_I(gi))->call_count++)

#define gc_stat_inc_seg_count(sbi, type)				\
	do {								\
		struct f2fs_gc_info *gi = sbi->gc_info;			\
		GC_STAT_I(gi)->tot_segs++;				\
		if (type == SUM_TYPE_DATA)				\
			GC_STAT_I(gi)->data_segs++;			\
		else							\
			GC_STAT_I(gi)->node_segs++;			\
	} while (0)

#define gc_stat_inc_tot_blk_count(gi, blks)				\
	((GC_STAT_I(gi)->tot_blks) += (blks))

#define gc_stat_inc_data_blk_count(sbi, blks)				\
	do {								\
		struct f2fs_gc_info *gi = sbi->gc_info;			\
		gc_stat_inc_tot_blk_count(gi, blks);			\
		GC_STAT_I(gi)->data_blks += (blks);			\
	} while (0)

#define gc_stat_inc_node_blk_count(sbi, blks)				\
	do {								\
		struct f2fs_gc_info *gi = sbi->gc_info;			\
		gc_stat_inc_tot_blk_count(gi, blks);			\
		GC_STAT_I(gi)->node_blks += (blks);			\
	} while (0)

#else
#define gc_stat_inc_call_count(gi)
#define gc_stat_inc_seg_count(gi, type)
#define gc_stat_inc_tot_blk_count(gi, blks)
#define gc_stat_inc_data_blk_count(gi, blks)
#define gc_stat_inc_node_blk_count(sbi, blks)
#endif

struct f2fs_gc_kthread {
	struct task_struct *f2fs_gc_task;
	wait_queue_head_t gc_wait_queue_head;
};

struct inode_entry {
	struct list_head list;
	struct inode *inode;
};

/**
 * inline functions
 */
static inline block_t free_user_blocks(struct f2fs_sb_info *sbi)
{
	if (free_segments(sbi) < overprovision_segments(sbi))
		return 0;
	else
		return (free_segments(sbi) - overprovision_segments(sbi))
			<< sbi->log_blocks_per_seg;
}

static inline block_t limit_invalid_user_blocks(struct f2fs_sb_info *sbi)
{
	return (long)(sbi->user_block_count * LIMIT_INVALID_BLOCK) / 100;
}

static inline block_t limit_free_user_blocks(struct f2fs_sb_info *sbi)
{
	block_t reclaimable_user_blocks = sbi->user_block_count -
		written_block_count(sbi);
	return (long)(reclaimable_user_blocks * LIMIT_FREE_BLOCK) / 100;
}

static inline long increase_sleep_time(long wait)
{
	wait += GC_THREAD_MIN_SLEEP_TIME;
	if (wait > GC_THREAD_MAX_SLEEP_TIME)
		wait = GC_THREAD_MAX_SLEEP_TIME;
	return wait;
}

static inline long decrease_sleep_time(long wait)
{
	wait -= GC_THREAD_MIN_SLEEP_TIME;
	if (wait <= GC_THREAD_MIN_SLEEP_TIME)
		wait = GC_THREAD_MIN_SLEEP_TIME;
	return wait;
}

static inline bool has_enough_invalid_blocks(struct f2fs_sb_info *sbi)
{
	block_t invalid_user_blocks = sbi->user_block_count -
					written_block_count(sbi);
	/*
	 * Background GC is triggered with the following condition.
	 * 1. There are a number of invalid blocks.
	 * 2. There is not enough free space.
	 */
	if (invalid_user_blocks > limit_invalid_user_blocks(sbi) &&
			free_user_blocks(sbi) < limit_free_user_blocks(sbi))
		return true;
	return false;
}

static inline int is_idle(struct f2fs_sb_info *sbi)
{
	struct block_device *bdev = sbi->sb->s_bdev;
	struct request_queue *q = bdev_get_queue(bdev);
	struct request_list *rl = &q->root_rl;
	return !(rl->count[BLK_RW_SYNC]) && !(rl->count[BLK_RW_ASYNC]);
}

static inline bool should_do_checkpoint(struct f2fs_sb_info *sbi)
{
	unsigned int pages_per_sec = sbi->segs_per_sec *
					(1 << sbi->log_blocks_per_seg);
	int node_secs = ((get_pages(sbi, F2FS_DIRTY_NODES) + pages_per_sec - 1)
			>> sbi->log_blocks_per_seg) / sbi->segs_per_sec;
	int dent_secs = ((get_pages(sbi, F2FS_DIRTY_DENTS) + pages_per_sec - 1)
			>> sbi->log_blocks_per_seg) / sbi->segs_per_sec;
	return free_sections(sbi) <= (node_secs + 2 * dent_secs + 2);
}
