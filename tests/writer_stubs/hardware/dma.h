#pragma once
#include <stdint.h>
struct dma_channel_config {};
constexpr unsigned DMA_SIZE_32 = 2;
int dma_claim_unused_channel(bool);
dma_channel_config dma_channel_get_default_config(unsigned);
void channel_config_set_transfer_data_size(dma_channel_config*, unsigned);
void channel_config_set_read_increment(dma_channel_config*, bool);
void channel_config_set_write_increment(dma_channel_config*, bool);
void channel_config_set_dreq(dma_channel_config*, unsigned);
void channel_config_set_high_priority(dma_channel_config*, bool);
void dma_channel_configure(unsigned, const dma_channel_config*, volatile void*, const void*, unsigned, bool);
bool dma_channel_is_busy(unsigned);
void dma_channel_abort(unsigned);
void dma_channel_unclaim(unsigned);
