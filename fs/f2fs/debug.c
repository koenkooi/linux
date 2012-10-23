/**
 * f2fs debugging statistics
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 * Copyright (c) 2012 Linux Foundation
 * Copyright (c) 2012 Greg Kroah-Hartman <gregkh@linuxfoundation.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/backing-dev.h>
#include <linux/proc_fs.h>
#include <linux/f2fs_fs.h>
#include <linux/blkdev.h>

#include "f2fs.h"
#include "node.h"
#include "segment.h"
#include "gc.h"

static LIST_HEAD(f2fs_stat_list);
static struct proc_dir_entry *f2fs_proc_root;


void f2fs_update_stat(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	struct f2fs_stat_info *si = gc_i->stat_info;
	int i;

	/* valid check of the segment numbers */
	si->hit_ext = sbi->read_hit_ext;
	si->total_ext = sbi->total_hit_ext;
	si->ndirty_node = get_pages(sbi, F2FS_DIRTY_NODES);
	si->ndirty_dent = get_pages(sbi, F2FS_DIRTY_DENTS);
	si->ndirty_dirs = sbi->n_dirty_dirs;
	si->ndirty_meta = get_pages(sbi, F2FS_DIRTY_META);
	si->total_count = (int)sbi->user_block_count / sbi->blocks_per_seg;
	si->rsvd_segs = reserved_segments(sbi);
	si->overp_segs = overprovision_segments(sbi);
	si->valid_count = valid_user_blocks(sbi);
	si->valid_node_count = valid_node_count(sbi);
	si->valid_inode_count = valid_inode_count(sbi);
	si->utilization = utilization(sbi);

	si->free_segs = free_segments(sbi);
	si->free_secs = free_sections(sbi);
	si->prefree_count = prefree_segments(sbi);
	si->dirty_count = dirty_segments(sbi);
	si->node_pages = sbi->node_inode->i_mapping->nrpages;
	si->meta_pages = sbi->meta_inode->i_mapping->nrpages;
	si->nats = NM_I(sbi)->nat_cnt;
	si->sits = SIT_I(sbi)->dirty_sentries;
	si->fnids = NM_I(sbi)->fcnt;
	si->bg_gc = sbi->bg_gc;
	si->util_free = (int)(free_user_blocks(sbi) >> sbi->log_blocks_per_seg)
		* 100 / (int)(sbi->user_block_count >> sbi->log_blocks_per_seg)
		/ 2;
	si->util_valid = (int)(written_block_count(sbi) >>
						sbi->log_blocks_per_seg)
		* 100 / (int)(sbi->user_block_count >> sbi->log_blocks_per_seg)
		/ 2;
	si->util_invalid = 50 - si->util_free - si->util_valid;
	for (i = CURSEG_HOT_DATA; i <= CURSEG_COLD_NODE; i++) {
		struct curseg_info *curseg = CURSEG_I(sbi, i);
		si->curseg[i] = curseg->segno;
		si->cursec[i] = curseg->segno / sbi->segs_per_sec;
		si->curzone[i] = si->cursec[i] / sbi->secs_per_zone;
	}

	for (i = 0; i < 2; i++) {
		si->segment_count[i] = sbi->segment_count[i];
		si->block_count[i] = sbi->block_count[i];
	}
}

/**
 * This function calculates BDF of every segments
 */
static void f2fs_update_gc_metric(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	struct f2fs_stat_info *si = gc_i->stat_info;
	unsigned int blks_per_sec, hblks_per_sec, total_vblocks, bimodal, dist;
	struct sit_info *sit_i = SIT_I(sbi);
	unsigned int segno, vblocks;
	int ndirty = 0;

	bimodal = 0;
	total_vblocks = 0;
	blks_per_sec = sbi->segs_per_sec * (1 << sbi->log_blocks_per_seg);
	hblks_per_sec = blks_per_sec / 2;
	mutex_lock(&sit_i->sentry_lock);
	for (segno = 0; segno < TOTAL_SEGS(sbi); segno += sbi->segs_per_sec) {
		vblocks = get_valid_blocks(sbi, segno, sbi->segs_per_sec);
		dist = abs(vblocks - hblks_per_sec);
		bimodal += dist * dist;

		if (vblocks > 0 && vblocks < blks_per_sec) {
			total_vblocks += vblocks;
			ndirty++;
		}
	}
	mutex_unlock(&sit_i->sentry_lock);
	dist = sbi->total_sections * hblks_per_sec * hblks_per_sec / 100;
	si->bimodal = bimodal / dist;
	if (si->dirty_count)
		si->avg_vblocks = total_vblocks / ndirty;
	else
		si->avg_vblocks = 0;
}

static int f2fs_read_gc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct f2fs_gc_info *gc_i, *next;
	struct f2fs_stat_info *si;
	char *buf = page;
	int i = 0;

	list_for_each_entry_safe(gc_i, next, &f2fs_stat_list, stat_list) {
		int j;
		si = gc_i->stat_info;

		mutex_lock(&si->stat_list);
		if (!si->sbi) {
			mutex_unlock(&si->stat_list);
			continue;
		}
		f2fs_update_stat(si->sbi);

		buf += sprintf(buf, "=====[ partition info. #%d ]=====\n", i++);
		buf += sprintf(buf, "[SB: 1] [CP: 2] [NAT: %d] [SIT: %d] ",
				si->nat_area_segs, si->sit_area_segs);
		buf += sprintf(buf, "[SSA: %d] [MAIN: %d",
				si->ssa_area_segs, si->main_area_segs);
		buf += sprintf(buf, "(OverProv:%d Resv:%d)]\n\n",
				si->overp_segs, si->rsvd_segs);
		buf += sprintf(buf, "Utilization: %d%% (%d valid blocks)\n",
				si->utilization, si->valid_count);
		buf += sprintf(buf, "  - Node: %u (Inode: %u, ",
				si->valid_node_count, si->valid_inode_count);
		buf += sprintf(buf, "Other: %u)\n  - Data: %u\n",
				si->valid_node_count - si->valid_inode_count,
				si->valid_count - si->valid_node_count);
		buf += sprintf(buf, "\nMain area: %d segs, %d secs %d zones\n",
				si->main_area_segs, si->main_area_sections,
				si->main_area_zones);
		buf += sprintf(buf, "  - COLD  data: %d, %d, %d\n",
				si->curseg[CURSEG_COLD_DATA],
				si->cursec[CURSEG_COLD_DATA],
				si->curzone[CURSEG_COLD_DATA]);
		buf += sprintf(buf, "  - WARM  data: %d, %d, %d\n",
				si->curseg[CURSEG_WARM_DATA],
				si->cursec[CURSEG_WARM_DATA],
				si->curzone[CURSEG_WARM_DATA]);
		buf += sprintf(buf, "  - HOT   data: %d, %d, %d\n",
				si->curseg[CURSEG_HOT_DATA],
				si->cursec[CURSEG_HOT_DATA],
				si->curzone[CURSEG_HOT_DATA]);
		buf += sprintf(buf, "  - Dir   dnode: %d, %d, %d\n",
				si->curseg[CURSEG_HOT_NODE],
				si->cursec[CURSEG_HOT_NODE],
				si->curzone[CURSEG_HOT_NODE]);
		buf += sprintf(buf, "  - File   dnode: %d, %d, %d\n",
				si->curseg[CURSEG_WARM_NODE],
				si->cursec[CURSEG_WARM_NODE],
				si->curzone[CURSEG_WARM_NODE]);
		buf += sprintf(buf, "  - Indir nodes: %d, %d, %d\n",
				si->curseg[CURSEG_COLD_NODE],
				si->cursec[CURSEG_COLD_NODE],
				si->curzone[CURSEG_COLD_NODE]);
		buf += sprintf(buf, "\n  - Valid: %d\n  - Dirty: %d\n",
				si->main_area_segs - si->dirty_count -
				si->prefree_count - si->free_segs,
				si->dirty_count);
		buf += sprintf(buf, "  - Prefree: %d\n  - Free: %d (%d)\n\n",
				si->prefree_count,
				si->free_segs,
				si->free_secs);
		buf += sprintf(buf, "GC calls: %d (BG: %d)\n",
				si->call_count, si->bg_gc);
		buf += sprintf(buf, "  - data segments : %d\n", si->data_segs);
		buf += sprintf(buf, "  - node segments : %d\n", si->node_segs);
		buf += sprintf(buf, "Try to move %d blocks\n", si->tot_blks);
		buf += sprintf(buf, "  - data blocks : %d\n", si->data_blks);
		buf += sprintf(buf, "  - node blocks : %d\n", si->node_blks);
		buf += sprintf(buf, "\nExtent Hit Ratio: %d / %d\n",
						si->hit_ext, si->total_ext);
		buf += sprintf(buf, "\nBalancing F2FS Async:\n");
		buf += sprintf(buf, "  - nodes %4d in %4d\n",
					si->ndirty_node, si->node_pages);
		buf += sprintf(buf, "  - dents %4d in dirs:%4d\n",
					si->ndirty_dent, si->ndirty_dirs);
		buf += sprintf(buf, "  - meta %4d in %4d\n",
					si->ndirty_meta, si->meta_pages);
		buf += sprintf(buf, "  - NATs %5d > %lu\n",
						si->nats, NM_WOUT_THRESHOLD);
		buf += sprintf(buf, "  - SITs: %5d\n  - free_nids: %5d\n",
					si->sits, si->fnids);
		buf += sprintf(buf, "\nDistribution of User Blocks:");
		buf += sprintf(buf, " [ valid | invalid | free ]\n");
		buf += sprintf(buf, "  [");
		for (j = 0; j < si->util_valid; j++)
			buf += sprintf(buf, "-");
		buf += sprintf(buf, "|");
		for (j = 0; j < si->util_invalid; j++)
			buf += sprintf(buf, "-");
		buf += sprintf(buf, "|");
		for (j = 0; j < si->util_free; j++)
			buf += sprintf(buf, "-");
		buf += sprintf(buf, "]\n\n");
		buf += sprintf(buf, "SSR: %u blocks in %u segments\n",
				si->block_count[SSR], si->segment_count[SSR]);
		buf += sprintf(buf, "LFS: %u blocks in %u segments\n",
				si->block_count[LFS], si->segment_count[LFS]);
		mutex_unlock(&si->stat_list);
	}
	return buf - page;
}

static int f2fs_read_sit(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct f2fs_gc_info *gc_i, *next;
	struct f2fs_stat_info *si;
	char *buf = page;

	list_for_each_entry_safe(gc_i, next, &f2fs_stat_list, stat_list) {
		si = gc_i->stat_info;

		mutex_lock(&si->stat_list);
		if (!si->sbi) {
			mutex_unlock(&si->stat_list);
			continue;
		}
		f2fs_update_gc_metric(si->sbi);

		buf += sprintf(buf, "BDF: %u, avg. vblocks: %u\n",
				si->bimodal, si->avg_vblocks);
		mutex_unlock(&si->stat_list);
	}
	return buf - page;
}

static int f2fs_read_mem(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct f2fs_gc_info *gc_i, *next;
	struct f2fs_stat_info *si;
	char *buf = page;

	list_for_each_entry_safe(gc_i, next, &f2fs_stat_list, stat_list) {
		struct f2fs_sb_info *sbi = gc_i->stat_info->sbi;
		unsigned npages;
		unsigned base_mem = 0, cache_mem = 0;

		si = gc_i->stat_info;
		mutex_lock(&si->stat_list);
		if (!si->sbi) {
			mutex_unlock(&si->stat_list);
			continue;
		}
		base_mem += sizeof(struct f2fs_sb_info) + sbi->sb->s_blocksize;
		base_mem += 2 * sizeof(struct f2fs_inode_info);
		base_mem += sizeof(*sbi->ckpt);

		/* build sm */
		base_mem += sizeof(struct f2fs_sm_info);

		/* build sit */
		base_mem += sizeof(struct sit_info);
		base_mem += TOTAL_SEGS(sbi) * sizeof(struct seg_entry);
		base_mem += f2fs_bitmap_size(TOTAL_SEGS(sbi));
		base_mem += 2 * SIT_VBLOCK_MAP_SIZE * TOTAL_SEGS(sbi);
		if (sbi->segs_per_sec > 1)
			base_mem += sbi->total_sections *
					sizeof(struct sec_entry);
		base_mem += __bitmap_size(sbi, SIT_BITMAP);

		/* build free segmap */
		base_mem += sizeof(struct free_segmap_info);
		base_mem += f2fs_bitmap_size(TOTAL_SEGS(sbi));
		base_mem += f2fs_bitmap_size(sbi->total_sections);

		/* build curseg */
		base_mem += sizeof(struct curseg_info) * NR_CURSEG_TYPE;
		base_mem += PAGE_CACHE_SIZE * NR_CURSEG_TYPE;

		/* build dirty segmap */
		base_mem += sizeof(struct dirty_seglist_info);
		base_mem += NR_DIRTY_TYPE * f2fs_bitmap_size(TOTAL_SEGS(sbi));
		base_mem += 2 * f2fs_bitmap_size(TOTAL_SEGS(sbi));

		/* buld nm */
		base_mem += sizeof(struct f2fs_nm_info);
		base_mem += __bitmap_size(sbi, NAT_BITMAP);

		/* build gc */
		base_mem += sizeof(struct f2fs_gc_info);
		base_mem += sizeof(struct f2fs_gc_kthread);

		/* free nids */
		cache_mem += NM_I(sbi)->fcnt;
		cache_mem += NM_I(sbi)->nat_cnt;
		npages = sbi->node_inode->i_mapping->nrpages;
		cache_mem += npages << PAGE_CACHE_SHIFT;
		npages = sbi->meta_inode->i_mapping->nrpages;
		cache_mem += npages << PAGE_CACHE_SHIFT;
		cache_mem += sbi->n_orphans * sizeof(struct orphan_inode_entry);
		cache_mem += sbi->n_dirty_dirs * sizeof(struct dir_inode_entry);

		buf += sprintf(buf, "%u KB = static: %u + cached: %u\n",
				(base_mem + cache_mem) >> 10,
				base_mem >> 10,
				cache_mem >> 10);
		mutex_unlock(&si->stat_list);
	}
	return buf - page;
}

static int init_stats(struct f2fs_sb_info *sbi)
{
	struct f2fs_stat_info *si;
	struct f2fs_super_block *raw_super = F2FS_RAW_SUPER(sbi);
	struct f2fs_gc_info *gc_i = sbi->gc_info;

	gc_i->stat_info = kzalloc(sizeof(struct f2fs_stat_info),
			GFP_KERNEL);
	if (!gc_i->stat_info)
		return -ENOMEM;
	si = gc_i->stat_info;
	mutex_init(&si->stat_list);
	list_add_tail(&gc_i->stat_list, &f2fs_stat_list);

	si->all_area_segs = le32_to_cpu(raw_super->segment_count);
	si->sit_area_segs = le32_to_cpu(raw_super->segment_count_sit);
	si->nat_area_segs = le32_to_cpu(raw_super->segment_count_nat);
	si->ssa_area_segs = le32_to_cpu(raw_super->segment_count_ssa);
	si->main_area_segs = le32_to_cpu(raw_super->segment_count_main);
	si->main_area_sections = le32_to_cpu(raw_super->section_count);
	si->main_area_zones = si->main_area_sections /
				le32_to_cpu(raw_super->secs_per_zone);
	si->sbi = sbi;
	return 0;
}

void f2fs_destroy_gci_stats(struct f2fs_gc_info *gc_i)
{
	struct f2fs_stat_info *si = gc_i->stat_info;

	list_del(&gc_i->stat_list);
	mutex_lock(&si->stat_list);
	si->sbi = NULL;
	mutex_unlock(&si->stat_list);
	kfree(gc_i->stat_info);
}

int f2fs_stat_init(struct super_block *sb, struct f2fs_sb_info *sbi)
{
	struct proc_dir_entry *entry;
	int retval;

	if (!f2fs_proc_root)
		f2fs_proc_root = proc_mkdir("fs/f2fs", NULL);

	sbi->s_proc = proc_mkdir(sb->s_id, f2fs_proc_root);

	retval = init_stats(sbi);
	if (retval)
		return retval;

	entry = create_proc_entry("f2fs_stat", 0, sbi->s_proc);
	if (!entry)
		return -ENOMEM;
	entry->read_proc = f2fs_read_gc;
	entry->write_proc = NULL;

	entry = create_proc_entry("f2fs_sit_stat", 0, sbi->s_proc);
	if (!entry) {
		remove_proc_entry("f2fs_stat", sbi->s_proc);
		return -ENOMEM;
	}
	entry->read_proc = f2fs_read_sit;
	entry->write_proc = NULL;
	entry = create_proc_entry("f2fs_mem_stat", 0, sbi->s_proc);
	if (!entry) {
		remove_proc_entry("f2fs_sit_stat", sbi->s_proc);
		remove_proc_entry("f2fs_stat", sbi->s_proc);
		return -ENOMEM;
	}
	entry->read_proc = f2fs_read_mem;
	entry->write_proc = NULL;
	return 0;
}

void f2fs_stat_exit(struct super_block *sb, struct f2fs_sb_info *sbi)
{
	if (sbi->s_proc) {
		remove_proc_entry("f2fs_stat", sbi->s_proc);
		remove_proc_entry("f2fs_sit_stat", sbi->s_proc);
		remove_proc_entry("f2fs_mem_stat", sbi->s_proc);
		remove_proc_entry(sb->s_id, f2fs_proc_root);
	}
}

void f2fs_remove_stats(void)
{
	remove_proc_entry("fs/f2fs", NULL);
}
