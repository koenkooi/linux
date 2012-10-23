/**
 * fs/f2fs/segment.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* constant macro */
#define NULL_SEGNO			((unsigned int)(~0))
#define SUM_TYPE_NODE			(1)
#define SUM_TYPE_DATA			(0)

/* V: Logical segment # in volume, R: Relative segment # in main area */
#define GET_L2R_SEGNO(free_i, segno)	(segno - free_i->start_segno)
#define GET_R2L_SEGNO(free_i, segno)	(segno + free_i->start_segno)

#define IS_DATASEG(t)							\
	((t == CURSEG_HOT_DATA) || (t == CURSEG_COLD_DATA) ||		\
	(t == CURSEG_WARM_DATA))

#define IS_NODESEG(t)							\
	((t == CURSEG_HOT_NODE) || (t == CURSEG_COLD_NODE) ||		\
	(t == CURSEG_WARM_NODE))

#define IS_CURSEG(sbi, segno)						\
	((segno == CURSEG_I(sbi, CURSEG_HOT_DATA)->segno) ||	\
	 (segno == CURSEG_I(sbi, CURSEG_WARM_DATA)->segno) ||	\
	 (segno == CURSEG_I(sbi, CURSEG_COLD_DATA)->segno) ||	\
	 (segno == CURSEG_I(sbi, CURSEG_HOT_NODE)->segno) ||	\
	 (segno == CURSEG_I(sbi, CURSEG_WARM_NODE)->segno) ||	\
	 (segno == CURSEG_I(sbi, CURSEG_COLD_NODE)->segno))

#define IS_CURSEC(sbi, secno)						\
	((secno == CURSEG_I(sbi, CURSEG_HOT_DATA)->segno /		\
	  sbi->segs_per_sec) ||	\
	 (secno == CURSEG_I(sbi, CURSEG_WARM_DATA)->segno /		\
	  sbi->segs_per_sec) ||	\
	 (secno == CURSEG_I(sbi, CURSEG_COLD_DATA)->segno /		\
	  sbi->segs_per_sec) ||	\
	 (secno == CURSEG_I(sbi, CURSEG_HOT_NODE)->segno /		\
	  sbi->segs_per_sec) ||	\
	 (secno == CURSEG_I(sbi, CURSEG_WARM_NODE)->segno /		\
	  sbi->segs_per_sec) ||	\
	 (secno == CURSEG_I(sbi, CURSEG_COLD_NODE)->segno /		\
	  sbi->segs_per_sec))	\

#define START_BLOCK(sbi, segno)						\
	(SM_I(sbi)->seg0_blkaddr +					\
	 (GET_R2L_SEGNO(FREE_I(sbi), segno) << sbi->log_blocks_per_seg))
#define NEXT_FREE_BLKADDR(sbi, curseg)					\
	(START_BLOCK(sbi, curseg->segno) + curseg->next_blkoff)

#define MAIN_BASE_BLOCK(sbi)	(SM_I(sbi)->main_blkaddr)

#define GET_SEGOFF_FROM_SEG0(sbi, blk_addr)				\
	((blk_addr) - SM_I(sbi)->seg0_blkaddr)
#define GET_SEGNO_FROM_SEG0(sbi, blk_addr)				\
	(GET_SEGOFF_FROM_SEG0(sbi, blk_addr) >> sbi->log_blocks_per_seg)
#define GET_SEGNO(sbi, blk_addr)					\
	(((blk_addr == NULL_ADDR) || (blk_addr == NEW_ADDR)) ?		\
	NULL_SEGNO : GET_L2R_SEGNO(FREE_I(sbi),			\
		GET_SEGNO_FROM_SEG0(sbi, blk_addr)))
#define GET_SECNO(sbi, segno)					\
	((segno) / sbi->segs_per_sec)
#define GET_ZONENO_FROM_SEGNO(sbi, segno)				\
	((segno / sbi->segs_per_sec) / sbi->secs_per_zone)

#define GET_SUM_BLOCK(sbi, segno)				\
	((sbi->sm_info->ssa_blkaddr) + segno)

#define GET_SUM_TYPE(footer) ((footer)->entry_type)
#define SET_SUM_TYPE(footer, type) ((footer)->entry_type = type)

#define SIT_ENTRY_OFFSET(sit_i, segno)					\
	(segno % sit_i->sents_per_block)
#define SIT_BLOCK_OFFSET(sit_i, segno)					\
	(segno / SIT_ENTRY_PER_BLOCK)
#define	START_SEGNO(sit_i, segno)		\
	(SIT_BLOCK_OFFSET(sit_i, segno) * SIT_ENTRY_PER_BLOCK)
#define f2fs_bitmap_size(nr)			\
	(BITS_TO_LONGS(nr) * sizeof(unsigned long))
#define TOTAL_SEGS(sbi)	(SM_I(sbi)->main_segment_count)

enum {
	LFS = 0,
	SSR
};

enum {
	ALLOC_RIGHT = 0,
	ALLOC_LEFT
};

#define SET_SSR_TYPE(type)	(((type) + 1) << 16)
#define GET_SSR_TYPE(type)	(((type) >> 16) - 1)
#define IS_SSR_TYPE(type)	((type) >= (0x1 << 16))
#define IS_NEXT_SEG(sbi, curseg, type)					\
	(DIRTY_I(sbi)->v_ops->get_victim(sbi, &(curseg)->next_segno,	\
				     BG_GC, SET_SSR_TYPE(type)))
/**
 * The MSB 6 bits of f2fs_sit_entry->vblocks has segment type,
 * and LSB 10 bits has valid blocks.
 */
#define VBLOCKS_MASK		((1 << 10) - 1)

#define GET_SIT_VBLOCKS(raw_sit)	\
	(le16_to_cpu((raw_sit)->vblocks) & VBLOCKS_MASK)
#define GET_SIT_TYPE(raw_sit)		\
	((le16_to_cpu((raw_sit)->vblocks) & ~VBLOCKS_MASK) >> 10)

struct bio_private {
	struct f2fs_sb_info *sbi;
	bool is_sync;
	void *wait;
};

enum {
	GC_CB = 0,
	GC_GREEDY
};

struct victim_sel_policy {
	int alloc_mode;
	int gc_mode;
	int type;
	unsigned long *dirty_segmap;
	unsigned int offset;
	unsigned int ofs_unit;
	unsigned int min_cost;
	unsigned int min_segno;
};

struct seg_entry {
	unsigned short valid_blocks;
	unsigned char *cur_valid_map;
	unsigned short ckpt_valid_blocks;
	unsigned char *ckpt_valid_map;
	unsigned char type;
	unsigned long long mtime;
};

struct sec_entry {
	unsigned int valid_blocks;
};

struct segment_allocation {
	void (*allocate_segment)(struct f2fs_sb_info *, int, bool);
};

struct sit_info {
	const struct segment_allocation *s_ops;

	block_t sit_base_addr;
	block_t sit_blocks;
	block_t written_valid_blocks;		/* total number of valid blocks
						   in main area */
	char *sit_bitmap;			/* SIT bitmap pointer */
	unsigned int bitmap_size;

	unsigned int dirty_sentries;		/* # of dirty sentries */
	unsigned long *dirty_sentries_bitmap;	/* bitmap for dirty sentries */
	unsigned int sents_per_block;		/* number of SIT entries
						   per SIT block */
	struct mutex sentry_lock;		/* to protect SIT entries */
	struct seg_entry *sentries;
	struct sec_entry *sec_entries;

	unsigned long long elapsed_time;
	unsigned long long mounted_time;
	unsigned long long min_mtime;
	unsigned long long max_mtime;
};

struct free_segmap_info {
	unsigned int start_segno;
	unsigned int free_segments;
	unsigned int free_sections;
	rwlock_t segmap_lock;		/* free segmap lock */
	unsigned long *free_segmap;
	unsigned long *free_secmap;
};

/* Notice: The order of dirty type is same with CURSEG_XXX in f2fs.h */
enum dirty_type {
	DIRTY_HOT_DATA,		/* a few valid blocks in a data segment */
	DIRTY_WARM_DATA,
	DIRTY_COLD_DATA,
	DIRTY_HOT_NODE,		/* a few valid blocks in a node segment */
	DIRTY_WARM_NODE,
	DIRTY_COLD_NODE,
	DIRTY,
	PRE,			/* no valid blocks in a segment */
	NR_DIRTY_TYPE
};

enum {
	BG_GC,
	FG_GC
};

struct dirty_seglist_info {
	const struct victim_selection *v_ops;
	struct mutex seglist_lock;
	unsigned long *dirty_segmap[NR_DIRTY_TYPE];
	int nr_dirty[NR_DIRTY_TYPE];
	unsigned long *victim_segmap[2];	/* BG_GC, FG_GC */
};

struct victim_selection {
	int (*get_victim)(struct f2fs_sb_info *, unsigned int *, int, int);
};

struct curseg_info {
	struct mutex curseg_mutex;
	struct f2fs_summary_block *sum_blk;
	unsigned char alloc_type;
	unsigned int segno;
	unsigned short next_blkoff;
	unsigned int zone;
	unsigned int next_segno;
};

/**
 * inline functions
 */
static inline struct curseg_info *CURSEG_I(struct f2fs_sb_info *sbi, int type)
{
	return (struct curseg_info *)(SM_I(sbi)->curseg_array + type);
}

static inline struct seg_entry *get_seg_entry(struct f2fs_sb_info *sbi,
						unsigned int segno)
{
	struct sit_info *sit_i = SIT_I(sbi);
	return &sit_i->sentries[segno];
}

static inline struct sec_entry *get_sec_entry(struct f2fs_sb_info *sbi,
						unsigned int segno)
{
	struct sit_info *sit_i = SIT_I(sbi);
	return &sit_i->sec_entries[GET_SECNO(sbi, segno)];
}

static inline unsigned int get_valid_blocks(struct f2fs_sb_info *sbi,
				unsigned int segno, int section)
{
	if (section > 1)
		return get_sec_entry(sbi, segno)->valid_blocks;
	else
		return get_seg_entry(sbi, segno)->valid_blocks;
}

static inline void seg_info_from_raw_sit(struct seg_entry *se,
					struct f2fs_sit_entry *rs)
{
	se->valid_blocks = GET_SIT_VBLOCKS(rs);
	se->ckpt_valid_blocks = GET_SIT_VBLOCKS(rs);
	memcpy(se->cur_valid_map, rs->valid_map, SIT_VBLOCK_MAP_SIZE);
	memcpy(se->ckpt_valid_map, rs->valid_map, SIT_VBLOCK_MAP_SIZE);
	se->type = GET_SIT_TYPE(rs);
	se->mtime = le64_to_cpu(rs->mtime);
}

static inline void seg_info_to_raw_sit(struct seg_entry *se,
					struct f2fs_sit_entry *rs)
{
	unsigned short raw_vblocks = (se->type << 10) | se->valid_blocks;
	rs->vblocks = cpu_to_le16(raw_vblocks);
	memcpy(rs->valid_map, se->cur_valid_map, SIT_VBLOCK_MAP_SIZE);
	memcpy(se->ckpt_valid_map, rs->valid_map, SIT_VBLOCK_MAP_SIZE);
	se->ckpt_valid_blocks = se->valid_blocks;
	rs->mtime = cpu_to_le64(se->mtime);
}

static inline unsigned int find_next_inuse(struct free_segmap_info *free_i,
		unsigned int max, unsigned int segno)
{
	unsigned int ret;
	read_lock(&free_i->segmap_lock);
	ret = find_next_bit(free_i->free_segmap, max, segno);
	read_unlock(&free_i->segmap_lock);
	return ret;
}

static inline void __set_free(struct f2fs_sb_info *sbi, unsigned int segno)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int secno = segno / sbi->segs_per_sec;
	unsigned int start_segno = secno * sbi->segs_per_sec;
	unsigned int next;

	write_lock(&free_i->segmap_lock);
	clear_bit(segno, free_i->free_segmap);
	free_i->free_segments++;

	next = find_next_bit(free_i->free_segmap, TOTAL_SEGS(sbi), start_segno);
	if (next >= start_segno + sbi->segs_per_sec) {
		clear_bit(secno, free_i->free_secmap);
		free_i->free_sections++;
	}
	write_unlock(&free_i->segmap_lock);
}

static inline void __set_inuse(struct f2fs_sb_info *sbi,
		unsigned int segno)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int secno = segno / sbi->segs_per_sec;
	set_bit(segno, free_i->free_segmap);
	free_i->free_segments--;
	if (!test_and_set_bit(secno, free_i->free_secmap))
		free_i->free_sections--;
}

static inline void __set_test_and_free(struct f2fs_sb_info *sbi,
		unsigned int segno)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int secno = segno / sbi->segs_per_sec;
	unsigned int start_segno = secno * sbi->segs_per_sec;
	unsigned int next;

	write_lock(&free_i->segmap_lock);
	if (test_and_clear_bit(segno, free_i->free_segmap)) {
		free_i->free_segments++;

		next = find_next_bit(free_i->free_segmap, TOTAL_SEGS(sbi),
								start_segno);
		if (next >= start_segno + sbi->segs_per_sec) {
			if (test_and_clear_bit(secno, free_i->free_secmap))
				free_i->free_sections++;
		}
	}
	write_unlock(&free_i->segmap_lock);
}

static inline void __set_test_and_inuse(struct f2fs_sb_info *sbi,
		unsigned int segno)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int secno = segno / sbi->segs_per_sec;
	write_lock(&free_i->segmap_lock);
	if (!test_and_set_bit(segno, free_i->free_segmap)) {
		free_i->free_segments--;
		if (!test_and_set_bit(secno, free_i->free_secmap))
			free_i->free_sections--;
	}
	write_unlock(&free_i->segmap_lock);
}

static inline void get_sit_bitmap(struct f2fs_sb_info *sbi,
		void *dst_addr)
{
	struct sit_info *sit_i = SIT_I(sbi);
	memcpy(dst_addr, sit_i->sit_bitmap, sit_i->bitmap_size);
}

static inline block_t written_block_count(struct f2fs_sb_info *sbi)
{
	struct sit_info *sit_i = SIT_I(sbi);
	block_t vblocks;

	mutex_lock(&sit_i->sentry_lock);
	vblocks = sit_i->written_valid_blocks;
	mutex_unlock(&sit_i->sentry_lock);

	return vblocks;
}

static inline unsigned int free_segments(struct f2fs_sb_info *sbi)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int free_segs;

	read_lock(&free_i->segmap_lock);
	free_segs = free_i->free_segments;
	read_unlock(&free_i->segmap_lock);

	return free_segs;
}

static inline int reserved_segments(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	return gc_i->rsvd_segment_count;
}

static inline unsigned int free_sections(struct f2fs_sb_info *sbi)
{
	struct free_segmap_info *free_i = FREE_I(sbi);
	unsigned int free_secs;

	read_lock(&free_i->segmap_lock);
	free_secs = free_i->free_sections;
	read_unlock(&free_i->segmap_lock);

	return free_secs;
}

static inline unsigned int prefree_segments(struct f2fs_sb_info *sbi)
{
	return DIRTY_I(sbi)->nr_dirty[PRE];
}

static inline unsigned int dirty_segments(struct f2fs_sb_info *sbi)
{
	return DIRTY_I(sbi)->nr_dirty[DIRTY_HOT_DATA] +
		DIRTY_I(sbi)->nr_dirty[DIRTY_WARM_DATA] +
		DIRTY_I(sbi)->nr_dirty[DIRTY_COLD_DATA] +
		DIRTY_I(sbi)->nr_dirty[DIRTY_HOT_NODE] +
		DIRTY_I(sbi)->nr_dirty[DIRTY_WARM_NODE] +
		DIRTY_I(sbi)->nr_dirty[DIRTY_COLD_NODE];
}

static inline int overprovision_segments(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	return gc_i->overp_segment_count;
}

static inline int overprovision_sections(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	return ((unsigned int) gc_i->overp_segment_count) / sbi->segs_per_sec;
}

static inline int reserved_sections(struct f2fs_sb_info *sbi)
{
	struct f2fs_gc_info *gc_i = sbi->gc_info;
	return ((unsigned int) gc_i->rsvd_segment_count) / sbi->segs_per_sec;
}

static inline bool need_SSR(struct f2fs_sb_info *sbi)
{
	return (free_sections(sbi) < overprovision_sections(sbi));
}

static inline bool has_not_enough_free_secs(struct f2fs_sb_info *sbi)
{
	return free_sections(sbi) <= reserved_sections(sbi);
}

static inline int utilization(struct f2fs_sb_info *sbi)
{
	return (long int)valid_user_blocks(sbi) * 100 /
			(long int)sbi->user_block_count;
}

/* Disable In-Place-Update by default */
#define MIN_IPU_UTIL		100
static inline bool need_inplace_update(struct inode *inode)
{
	struct f2fs_sb_info *sbi = F2FS_SB(inode->i_sb);
	if (S_ISDIR(inode->i_mode))
		return false;
	if (need_SSR(sbi) && utilization(sbi) > MIN_IPU_UTIL)
		return true;
	return false;
}

static inline unsigned int curseg_segno(struct f2fs_sb_info *sbi,
		int type)
{
	struct curseg_info *curseg = CURSEG_I(sbi, type);
	return curseg->segno;
}

static inline unsigned char curseg_alloc_type(struct f2fs_sb_info *sbi,
		int type)
{
	struct curseg_info *curseg = CURSEG_I(sbi, type);
	return curseg->alloc_type;
}

static inline unsigned short curseg_blkoff(struct f2fs_sb_info *sbi, int type)
{
	struct curseg_info *curseg = CURSEG_I(sbi, type);
	return curseg->next_blkoff;
}

static inline void check_seg_range(struct f2fs_sb_info *sbi, unsigned int segno)
{
	unsigned int end_segno = SM_I(sbi)->segment_count - 1;
	BUG_ON(segno > end_segno);
}

/*
 * This function is used for only debugging.
 * NOTE: In future, we have to remove this function.
 */
static inline void verify_block_addr(struct f2fs_sb_info *sbi, block_t blk_addr)
{
	struct f2fs_sm_info *sm_info = SM_I(sbi);
	block_t total_blks = sm_info->segment_count << sbi->log_blocks_per_seg;
	block_t start_addr = sm_info->seg0_blkaddr;
	block_t end_addr = start_addr + total_blks - 1;
	BUG_ON(blk_addr < start_addr);
	BUG_ON(blk_addr > end_addr);
}

/**
 * Summary block is always treated as invalid block
 */
static inline void check_block_count(struct f2fs_sb_info *sbi,
		int segno, struct f2fs_sit_entry *raw_sit)
{
	struct f2fs_sm_info *sm_info = SM_I(sbi);
	unsigned int end_segno = sm_info->segment_count - 1;
	int valid_blocks = 0;
	int i;

	/* check segment usage */
	BUG_ON(GET_SIT_VBLOCKS(raw_sit) > sbi->blocks_per_seg);

	/* check boundary of a given segment number */
	BUG_ON(segno > end_segno);

	/* check bitmap with valid block count */
	for (i = 0; i < sbi->blocks_per_seg; i++)
		if (f2fs_test_bit(i, raw_sit->valid_map))
			valid_blocks++;
	BUG_ON(GET_SIT_VBLOCKS(raw_sit) != valid_blocks);
}

static inline pgoff_t current_sit_addr(struct f2fs_sb_info *sbi,
						unsigned int start)
{
	struct sit_info *sit_i = SIT_I(sbi);
	unsigned int offset = SIT_BLOCK_OFFSET(sit_i, start);
	block_t blk_addr = sit_i->sit_base_addr + offset;

	check_seg_range(sbi, start);

	/* calculate sit block address */
	if (f2fs_test_bit(offset, sit_i->sit_bitmap))
		blk_addr += sit_i->sit_blocks;

	return blk_addr;
}

static inline pgoff_t next_sit_addr(struct f2fs_sb_info *sbi,
						pgoff_t block_addr)
{
	struct sit_info *sit_i = SIT_I(sbi);
	block_addr -= sit_i->sit_base_addr;
	if (block_addr < sit_i->sit_blocks)
		block_addr += sit_i->sit_blocks;
	else
		block_addr -= sit_i->sit_blocks;

	return block_addr + sit_i->sit_base_addr;
}

static inline void set_to_next_sit(struct sit_info *sit_i, unsigned int start)
{
	unsigned int block_off = SIT_BLOCK_OFFSET(sit_i, start);

	if (f2fs_test_bit(block_off, sit_i->sit_bitmap))
		f2fs_clear_bit(block_off, sit_i->sit_bitmap);
	else
		f2fs_set_bit(block_off, sit_i->sit_bitmap);
}

static inline unsigned long long get_mtime(struct f2fs_sb_info *sbi)
{
	struct sit_info *sit_i = SIT_I(sbi);
	return sit_i->elapsed_time + CURRENT_TIME_SEC.tv_sec -
						sit_i->mounted_time;
}

static inline void set_summary(struct f2fs_summary *sum, nid_t nid,
			unsigned int ofs_in_node, unsigned char version)
{
	sum->nid = cpu_to_le32(nid);
	sum->ofs_in_node = cpu_to_le16(ofs_in_node);
	sum->version = version;
}

static inline block_t start_sum_block(struct f2fs_sb_info *sbi)
{
	return __start_cp_addr(sbi) +
		le32_to_cpu(F2FS_CKPT(sbi)->cp_pack_start_sum);
}

static inline block_t sum_blk_addr(struct f2fs_sb_info *sbi, int base, int type)
{
	return __start_cp_addr(sbi) +
		le32_to_cpu(F2FS_CKPT(sbi)->cp_pack_total_block_count)
				- (base + 1) + type;
}
