#ifndef RRTSTAR_RENDERER_H_
#define RRTSTAR_RENDERER_H_
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_renderer_rrtstar (BotViewer *viewer, int priority, lcm_t *_lcm);

#ifdef __cplusplus
}
#endif

#endif
