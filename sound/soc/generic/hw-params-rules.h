/*
 * ASoC generic hw_params_rules support
 *
 * Copyright (C) 2016 Martin Sperl
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HW_PARAMS_RULES_H
#define __HW_PARAMS_RULES_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/of.h>
#include <sound/pcm.h>

#if defined(CONFIG_SND_HW_PARAMS_RULES) ||	\
	defined(CONFIG_SND_HW_PARAMS_RULES_MODULE)

int asoc_generic_hw_params_rules_parse_of(
	struct device *dev,
	struct device_node *node,
	struct list_head *list_head);

int asoc_generic_hw_params_process_rules(
	struct list_head *list_head,
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params);
#else
static int asoc_generic_hw_params_rules_parse_of(
	struct device *dev,
	struct device_node *node,
	struct list_head *list_head)
{
	return 0;
}

static int asoc_generic_hw_params_process_rules(
	struct list_head *list_head,
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}
#endif

#endif
