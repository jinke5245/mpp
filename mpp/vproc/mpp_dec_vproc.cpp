/*
 * Copyright 2018 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define MODULE_TAG "mpp_dec_vproc"

#include "mpp_env.h"
#include "mpp_mem.h"
#include "mpp_common.h"

#include "mpp_frame_impl.h"
#include "mpp_dec_vproc.h"
#include "iep_api.h"

#define vproc_dbg(flag, fmt, ...) \
    do { \
        _mpp_dbg(vproc_debug, flag, fmt, ## __VA_ARGS__); \
    } while (0)

#define vproc_dbg_f(flag, fmt, ...) \
    do { \
        _mpp_dbg_f(vproc_debug, flag, fmt, ## __VA_ARGS__); \
    } while (0)

#define VPROC_DBG_FUNCTION      (0x00000001)

#define vproc_func(fmt, ...)  \
    vproc_dbg_f(VPROC_DBG_FUNCTION, fmt, ## __VA_ARGS__);

RK_U32 vproc_debug = 0;

typedef struct MppDecVprocCtxImpl_t {
    Mpp         *mpp;
    MppBufSlots slots;

    MppThread   *thd;
    RK_U32      reset;
    RK_S32      count;

    IepCtx      iep_ctx;
} MppDecVprocCtxImpl;

static void mpp_enqueue_frames(Mpp *mpp, MppFrame frame)
{
    mpp_list *list = mpp->mFrames;

    list->lock();
    list->add_at_tail(&frame, sizeof(frame));

    if (mpp_debug & MPP_DBG_PTS)
        mpp_log("output frame pts %lld\n", mpp_frame_get_pts(frame));

    mpp->mFramePutCount++;
    list->signal();
    list->unlock();
}

static void *dec_vproc_thread(void *data)
{
    MppDecVprocCtxImpl *ctx = (MppDecVprocCtxImpl *)data;
    MppThread *thd = ctx->thd;
    Mpp *mpp = ctx->mpp;
    MppDec *dec = mpp->mDec;
    MppBufSlots slots = dec->frame_slots;
    IepImg img;

    mpp_dbg(MPP_DBG_INFO, "mpp_dec_post_proc_thread started\n");

    while (1) {
        RK_S32 index = -1;
        MPP_RET ret = MPP_OK;

        {
            AutoMutex autolock(thd->mutex());

            if (MPP_THREAD_RUNNING != thd->get_status())
                break;

            if (ctx->reset) {
                // on reset just return all index
                do {
                    ret = mpp_buf_slot_dequeue(slots, &index, QUEUE_DEINTERLACE);
                    if (MPP_OK == ret && index >= 0) {
                        mpp_buf_slot_clr_flag(slots, index, SLOT_QUEUE_USE);
                        ctx->count--;
                        mpp_log_f("reset index %d\n", index);
                    }
                } while (ret == MPP_OK);
                mpp_assert(ctx->count == 0);

                thd->lock(THREAD_CONTROL);
                ctx->reset = 0;
                thd->signal(THREAD_CONTROL);
                thd->unlock(THREAD_CONTROL);
            } else {
                ret = mpp_buf_slot_dequeue(slots, &index, QUEUE_DEINTERLACE);
                if (ret)
                    thd->wait();
                else
                    ctx->count--;
            }

            if (ret)
                continue;
        }


        // dequeue from deinterlace queue then send to mpp->mFrames
        if (index >= 0) {
            MppFrame frame = NULL;
            mpp_buf_slot_get_prop(slots, index, SLOT_FRAME, &frame);
            if (mpp_frame_get_info_change(frame)) {
                mpp_enqueue_frames(mpp, frame);
                goto VPROC_DONE;
            }

            if (mpp_frame_get_eos(frame) &&
                NULL == mpp_frame_get_buffer(frame)) {
                mpp_enqueue_frames(mpp, frame);
                goto VPROC_DONE;
            }

            if (!dec->reset_flag && ctx->iep_ctx) {
                MppBufferGroup group = mpp->mFrameGroup;
                RK_U32 w = mpp_frame_get_width(frame);
                RK_U32 h = mpp_frame_get_height(frame);
                RK_U32 h_str = mpp_frame_get_hor_stride(frame);
                RK_U32 v_str = mpp_frame_get_ver_stride(frame);
                MppBuffer buf = mpp_frame_get_buffer(frame);
                int fd = mpp_buffer_get_fd(buf);
                size_t size = mpp_buffer_get_size(buf);
                MppFrame out = NULL;
                MppFrameImpl *impl = NULL;

                ret = iep_control(ctx->iep_ctx, IEP_CMD_INIT, NULL);
                if (ret)
                    mpp_log_f("IEP_CMD_INIT failed %d\n", ret);

                // setup source IepImg
                memset(&img, 0, sizeof(img));
                img.act_w = w;
                img.act_h = h;
                img.vir_w = h_str;
                img.vir_h = v_str;
                img.format = IEP_FORMAT_YCbCr_420_SP;
                img.mem_addr = fd;
                img.uv_addr = fd + ((h_str * v_str) << 10);
                img.v_addr = fd + ((h_str * v_str + h_str * v_str / 4) << 10);

                ret = iep_control(ctx->iep_ctx, IEP_CMD_SET_SRC, &img);
                if (ret)
                    mpp_log_f("IEP_CMD_SET_SRC failed %d\n", ret);

                // setup destination IepImg with new buffer
                buf = NULL;
                do {
                    mpp_buffer_get(group, &buf, size);
                    if (NULL == buf) {
                        mpp_log("failed to get buffer\n");
                        usleep(10000);
                    }
                } while (NULL == buf);
                mpp_assert(buf);

                fd = mpp_buffer_get_fd(buf);
                img.mem_addr = fd;
                img.uv_addr = fd + ((h_str * v_str) << 10);
                img.v_addr = fd + ((h_str * v_str + h_str * v_str / 4) << 10);

                ret = iep_control(ctx->iep_ctx, IEP_CMD_SET_DST, &img);
                if (ret)
                    mpp_log_f("IEP_CMD_SET_DST failed %d\n", ret);

                // start deinterlace hardware
                ret = iep_control(ctx->iep_ctx, IEP_CMD_SET_DEI_CFG, NULL);
                if (ret)
                    mpp_log_f("IEP_CMD_SET_DEI_CFG failed %d\n", ret);

                ret = iep_control(ctx->iep_ctx, IEP_CMD_RUN_SYNC, NULL);
                if (ret)
                    mpp_log_f("IEP_CMD_RUN_SYNC failed %d\n", ret);

                // generate new MppFrame for output
                mpp_frame_init(&out);
                mpp_frame_copy(out, frame);
                impl = (MppFrameImpl *)out;
                impl->buffer = buf;

                mpp_enqueue_frames(mpp, out);
            }
            mpp_frame_deinit(&frame);

        VPROC_DONE:
            mpp_buf_slot_clr_flag(slots, index, SLOT_QUEUE_USE);
        }
    }
    mpp_dbg(MPP_DBG_INFO, "mpp_dec_post_proc_thread exited\n");

    return NULL;
}

MPP_RET dec_vproc_init(MppDecVprocCtx *ctx, void *mpp)
{
    MPP_RET ret = MPP_OK;
    if (NULL == ctx || NULL == mpp) {
        mpp_err_f("found NULL input ctx %p mpp %p\n", ctx, mpp);
        return MPP_ERR_NULL_PTR;
    }

    vproc_func("in");
    mpp_env_get_u32("vproc_debug", &vproc_debug, 0);

    *ctx = NULL;

    MppDecVprocCtxImpl *p = mpp_calloc(MppDecVprocCtxImpl, 1);
    if (NULL == p) {
        mpp_err_f("malloc failed\n");
        return MPP_ERR_MALLOC;
    }

    p->mpp = (Mpp *)mpp;
    p->slots = p->mpp->mDec->frame_slots;
    p->thd = new MppThread(dec_vproc_thread, p, "mpp_dec_vproc");
    ret = iep_init(&p->iep_ctx);
    if (!p->thd || ret) {
        mpp_err("failed to create context\n");
        if (p->thd) {
            delete p->thd;
            p->thd = NULL;
        }

        if (p->iep_ctx) {
            iep_deinit(p->iep_ctx);
            p->iep_ctx = NULL;
        }

        MPP_FREE(p);
    }

    *ctx = p;

    vproc_func("out");
    return ret;
}

MPP_RET dec_vproc_deinit(MppDecVprocCtx ctx)
{
    if (NULL == ctx) {
        mpp_err_f("found NULL input\n");
        return MPP_ERR_NULL_PTR;
    }
    vproc_func("in");

    MppDecVprocCtxImpl *p = (MppDecVprocCtxImpl *)ctx;
    if (p->thd) {
        p->thd->stop();
        delete p->thd;
        p->thd = NULL;
    }

    if (p->iep_ctx) {
        iep_deinit(p->iep_ctx);
        p->iep_ctx = NULL;
    }

    mpp_free(p);

    vproc_func("out");
    return MPP_OK;
}

MPP_RET dec_vproc_start(MppDecVprocCtx ctx)
{
    if (NULL == ctx) {
        mpp_err_f("found NULL input\n");
        return MPP_ERR_NULL_PTR;
    }
    vproc_func("in");

    MppDecVprocCtxImpl *p = (MppDecVprocCtxImpl *)ctx;

    if (p->thd)
        p->thd->start();
    else
        mpp_err("failed to start dec vproc thread\n");

    vproc_func("out");
    return MPP_OK;
}

MPP_RET dec_vproc_stop(MppDecVprocCtx ctx)
{
    if (NULL == ctx) {
        mpp_err_f("found NULL input\n");
        return MPP_ERR_NULL_PTR;
    }
    vproc_func("in");

    MppDecVprocCtxImpl *p = (MppDecVprocCtxImpl *)ctx;

    if (p->thd)
        p->thd->stop();
    else
        mpp_err("failed to stop dec vproc thread\n");

    vproc_func("out");
    return MPP_OK;
}

MPP_RET dec_vproc_signal(MppDecVprocCtx ctx)
{
    if (NULL == ctx) {
        mpp_err_f("found NULL input\n");
        return MPP_ERR_NULL_PTR;
    }
    vproc_func("in");

    MppDecVprocCtxImpl *p = (MppDecVprocCtxImpl *)ctx;
    if (p->thd) {
        p->thd->lock();
        p->count++;
        p->thd->signal();
        p->thd->unlock();
    }

    vproc_func("out");
    return MPP_OK;
}

MPP_RET dec_vproc_reset(MppDecVprocCtx ctx)
{
    if (NULL == ctx) {
        mpp_err_f("found NULL input\n");
        return MPP_ERR_NULL_PTR;
    }
    vproc_func("in");

    MppDecVprocCtxImpl *p = (MppDecVprocCtxImpl *)ctx;
    if (p->thd) {
        p->thd->lock();
        p->reset = 1;
        p->thd->signal();
        p->thd->unlock();

        // wait reset finished
        p->thd->lock(THREAD_CONTROL);
        p->thd->wait(THREAD_CONTROL);
        p->thd->unlock(THREAD_CONTROL);

        mpp_assert(p->reset == 0);
    }

    vproc_func("out");
    return MPP_OK;
}
