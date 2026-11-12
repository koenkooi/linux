/* SPDX-License-Identifier: GPL-2.0-only */
/* Exports from core.c to the other pieces of the orinoco module.
 *
 * See copyright notice in core.c.
 */
#ifndef _ORINOCO_MAIN_H_
#define _ORINOCO_MAIN_H_

/* Forward declarations */
struct orinoco_private;
struct work_struct;

void set_port_type(struct orinoco_private *priv);
int orinoco_commit(struct orinoco_private *priv);
void orinoco_reset(struct work_struct *work);

#endif /* _ORINOCO_MAIN_H_ */
