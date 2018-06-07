#include "LXPlayerCore.h"



/* Since we only have one decoding thread, the Big Struct
can be global in case we need it. */

SDL_Window  *screen;
SDL_Renderer *renderer;
SDL_Texture *texture;
SDL_Rect showrect;
SDL_mutex   *screen_mutex;
SDL_mutex   *frame_mutex;
QList<Frame>    listFrame;
Frame			disFrame;
AVFrame *showFrame;
unsigned char *out_buffer;

int eof;
int num_putin_list = 0;

int num_decoded = 0;

int bufferIsReady = 1;
AVPacket flush_pkt;
/* Since we only have one decoding thread, the Big Struct
can be global in case we need it. */
VideoState *global_video_state;

void packet_queue_init(PacketQueue *q) {
	memset(q, 0, sizeof(PacketQueue));
	q->mutex = SDL_CreateMutex();
	q->cond = SDL_CreateCond();
}

int packet_queue_put(PacketQueue *q, AVPacket *pkt) {

	if (q->abort_request)
		return -1;
	AVPacketList *pkt1;
	if (pkt != &flush_pkt && av_dup_packet(pkt) < 0) {
		return -1;
	}
	pkt1 = (AVPacketList *)av_malloc(sizeof(AVPacketList));
	if (!pkt1)
		return -1;
	pkt1->pkt = *pkt;
	pkt1->next = NULL;

	SDL_LockMutex(q->mutex);

	if (!q->last_pkt)
		q->first_pkt = pkt1;
	else
		q->last_pkt->next = pkt1;
	q->last_pkt = pkt1;
	q->nb_packets++;
	q->size += pkt1->pkt.size;
	SDL_CondSignal(q->cond);

	SDL_UnlockMutex(q->mutex);
	return 0;
}
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block, int quit)
{
	if (q->abort_request)
		return -1;
	AVPacketList *pkt1;
	int ret;

	SDL_LockMutex(q->mutex);

	for (;;) {
		if (quit) {
			ret = -1;
			break;
		}
		pkt1 = q->first_pkt;
		if (pkt1) {
			q->first_pkt = pkt1->next;
			if (!q->first_pkt)
				q->last_pkt = NULL;
			q->nb_packets--;
			q->size -= pkt1->pkt.size;
			*pkt = pkt1->pkt;
			av_free(pkt1);
			ret = 1;
			break;
		}
		else if (!block) {
			ret = 0;
			break;
		}
		else {
			SDL_CondWait(q->cond, q->mutex);
		}
	}
	SDL_UnlockMutex(q->mutex);
	return ret;
}

static void packet_queue_flush(PacketQueue *q) {
	AVPacketList *pkt, *pkt1;

	SDL_LockMutex(q->mutex);
	for (pkt = q->first_pkt; pkt != NULL; pkt = pkt1) {
		pkt1 = pkt->next;
		av_free_packet(&pkt->pkt);
		av_freep(&pkt);
	}
	q->last_pkt = NULL;
	q->first_pkt = NULL;
	q->nb_packets = 0;
	q->size = 0;
	SDL_UnlockMutex(q->mutex);
}

static int packet_queue_abort(PacketQueue *q)
{
	SDL_LockMutex(q->mutex);
	q->abort_request = 1;
	SDL_CondSignal(q->cond);
	SDL_UnlockMutex(q->mutex);
	return 0;
}

double get_audio_clock(VideoState *is) {
	double pts;
	int hw_buf_size, bytes_per_sec, n;

	pts = is->audio_clock; /* maintained in the audio thread */
	hw_buf_size = is->audio_buf_size - is->audio_buf_index;
	bytes_per_sec = 0;
	n = is->audio_ctx->channels * 2;
	if (is->audio_st) {
		bytes_per_sec = is->audio_ctx->sample_rate * n;
	}
	if (bytes_per_sec) {
		pts -= (double)hw_buf_size / bytes_per_sec;
	}
	return pts;
}

/*
* @brief 获取video时钟
* @detail delta是当前时间减去上一帧显示的时间差
再用上一帧的pts加上这个时间差，则为video的时钟
*/
double get_video_clock(VideoState *is) {
	double delta;

	delta = (av_gettime() - is->video_current_pts_time) / 1000000.0;
	return is->video_current_pts + delta;
}
double get_external_clock(VideoState *is) {
	return av_gettime() / 1000000.0;
}

double get_master_clock(VideoState *is) {
	if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
		return get_video_clock(is);
	}
	else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
		return get_audio_clock(is);
	}
	else {
		return get_external_clock(is);
	}
}


/* Add or subtract samples to get a better sync, return new
audio buffer size */
int synchronize_audio(VideoState *is, short *samples,
	int samples_size, double pts) {
	int n;
	double ref_clock;

	n = 2 * is->audio_ctx->channels;

	if (is->av_sync_type != AV_SYNC_AUDIO_MASTER) {
		double diff, avg_diff;
		int wanted_size, min_size, max_size /*, nb_samples */;

		ref_clock = get_master_clock(is);
		diff = get_audio_clock(is) - ref_clock;

		if (diff < AV_NOSYNC_THRESHOLD) {
			// accumulate the diffs
			is->audio_diff_cum = diff + is->audio_diff_avg_coef
				* is->audio_diff_cum;
			if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
				is->audio_diff_avg_count++;
			}
			else {
				avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);
				if (fabs(avg_diff) >= is->audio_diff_threshold) {
					wanted_size = samples_size + ((int)(diff * is->audio_ctx->sample_rate) * n);
					min_size = samples_size * ((100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100);
					max_size = samples_size * ((100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100);
					if (wanted_size < min_size) {
						wanted_size = min_size;
					}
					else if (wanted_size > max_size) {
						wanted_size = max_size;
					}
					if (wanted_size < samples_size) {
						/* remove samples */
						samples_size = wanted_size;
					}
					else if (wanted_size > samples_size) {
						uint8_t *samples_end, *q;
						int nb;

						/* add samples by copying final sample*/
						nb = (samples_size - wanted_size);
						samples_end = (uint8_t *)samples + samples_size - n;
						q = samples_end + n;
						while (nb > 0) {
							memcpy(q, samples_end, n);
							q += n;
							nb -= n;
						}
						samples_size = wanted_size;
					}
				}
			}
		}
		else {
			/* difference is TOO big; reset diff stuff */
			is->audio_diff_avg_count = 0;
			is->audio_diff_cum = 0;
		}
	}
	return samples_size;
}

int audio_decode_frame(VideoState *is, uint8_t *audio_buf, int buf_size, double *pts_ptr) {

#if 1
	int len1, data_size = 0;
	AVPacket *pkt = &is->audio_pkt;
	double pts;
	int n;

	for (;;)
	{
		while (is->audio_pkt_size > 0)
		{
			int got_frame = 0;
			len1 = avcodec_decode_audio4(is->audio_ctx, &is->audio_frame, &got_frame, pkt);//len1指的是我们一次使用的大小，即一帧的大小
			if (len1 < 0) {
				/* if error, skip frame */
				is->audio_pkt_size = 0;
				break;
			}
			data_size = 0;
			if (got_frame) {
				data_size = av_samples_get_buffer_size(NULL,
					is->audio_ctx->channels,
					is->audio_frame.nb_samples,
					is->audio_ctx->sample_fmt,
					1);//data_size指的是整个packet的大小。
				assert(data_size <= buf_size);
				memcpy(audio_buf, is->audio_frame.data[0], data_size);
			}
			is->audio_pkt_data += len1;
			is->audio_pkt_size -= len1;
			if (data_size <= 0) {
				/* No data yet, get more frames */
				continue;
			}
			pts = is->audio_clock;
			*pts_ptr = pts;
			n = 2 * is->audio_ctx->channels;
			is->audio_clock += (double)data_size /
				(double)(n * is->audio_ctx->sample_rate);
			/* We have data, return it and come back for more later */
			return data_size;
					}
		if (pkt->data)
			av_free_packet(pkt);

		if (is->quit) {
			return -1;
		}
		/* next packet */
		if (packet_queue_get(&is->audioq, pkt, 1, is->quit) < 0) {
			return -1;
		}
		if (pkt->data == flush_pkt.data) {
			avcodec_flush_buffers(is->audio_ctx);
			continue;
		}
		is->audio_pkt_data = pkt->data;
		is->audio_pkt_size = pkt->size;
		/* if update, update the audio clock w/pts */
		if (pkt->pts != AV_NOPTS_VALUE) {
			is->audio_clock = av_q2d(is->audio_st->time_base)*pkt->pts;
		}
}
#endif

#if 0
	AVFrame *frame = av_frame_alloc();
	int data_size = 0;
	AVPacket *pkt = av_packet_alloc();
	SwrContext *swr_ctx = nullptr;
	static double clock = 0;

	if (packet_queue_get(&is->audioq, pkt, 0, is->quit) < 0)
	{
		goto fail;
	}
	if (pkt->pts != AV_NOPTS_VALUE)
	{
		is->audio_clock = av_q2d(is->pFormatCtx->streams[is->audioStream]->time_base) * pkt->pts;
	}
	if (is->audio_ctx == nullptr)
		goto fail;
	pkt->size;
	int ret = avcodec_send_packet(is->audio_ctx, pkt);
	if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
		goto fail;
	ret = avcodec_receive_frame(is->audio_ctx, frame);
	if (ret < 0 && ret != AVERROR_EOF)
		goto fail;
	/*data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(frame),
		frame->nb_samples, (AVSampleFormat)(frame->format), 0);*/

	// 设置通道数或channel_layout
	if (frame->channels > 0 && frame->channel_layout == 0)
		frame->channel_layout = av_get_default_channel_layout(frame->channels);
	else if (frame->channels == 0 && frame->channel_layout > 0)
		frame->channels = av_get_channel_layout_nb_channels(frame->channel_layout);

	AVSampleFormat dst_format = AV_SAMPLE_FMT_S16;//av_get_packed_sample_fmt((AVSampleFormat)frame->format);
	Uint64 dst_layout = av_get_default_channel_layout(frame->channels);
	// 设置转换参数
	swr_ctx = swr_alloc_set_opts(nullptr, dst_layout, dst_format, frame->sample_rate,
		frame->channel_layout, (AVSampleFormat)frame->format, frame->sample_rate, 0, nullptr);
	if (!swr_ctx || swr_init(swr_ctx) < 0)
		goto fail;
	// 计算转换后的sample个数 a * b / c
	uint64_t dst_nb_samples = av_rescale_rnd(swr_get_delay(swr_ctx, frame->sample_rate) + frame->nb_samples, frame->sample_rate, frame->sample_rate, AVRounding(1));
	// 转换，返回值为转换后的sample个数
	int nb = swr_convert(swr_ctx, &audio_buf, static_cast<int>(dst_nb_samples), (const uint8_t**)frame->data, frame->nb_samples);
	//data_size = frame->channels * nb * av_get_bytes_per_sample(dst_format);

	data_size = av_samples_get_buffer_size(NULL, av_frame_get_channels(frame),
		frame->nb_samples, (AVSampleFormat)(frame->format), 0);

	// 每秒钟音频播放的字节数 sample_rate * channels * sample_format(一个sample占用的字节数)
	//is->audio_clock += static_cast<double>(data_size) / (2 * is->stream->codec->channels * is->stream->codec->sample_rate);
	is->audio_clock += static_cast<double>(data_size) / (2 * is->audio_ctx->channels * is->audio_ctx->sample_rate);
	av_frame_free(&frame);
	swr_free(&swr_ctx);

	return data_size;

fail:
	av_frame_free(&frame);
	swr_free(&swr_ctx);
	return -1;
#endif
}

void audio_callback(void *userdata, Uint8 *stream, int len) {

	VideoState *is = (VideoState *)userdata;
	int len1, audio_size;
	double pts;
	SDL_memset(stream, 0, len);

	while (len > 0) {
		if (is->audio_buf_index >= is->audio_buf_size) {
			/* We have already sent all our data; get more */
			audio_size = audio_decode_frame(is, is->audio_buf, sizeof(is->audio_buf), &pts);
			if (audio_size < 0) {
				/* If error, output silence */
				is->audio_buf_size = 1024;
				memset(is->audio_buf, 0, is->audio_buf_size);
			}
			else {
				audio_size = synchronize_audio(is, (int16_t *)is->audio_buf,
					audio_size, pts);
				is->audio_buf_size = audio_size;
			}
			is->audio_buf_index = 0;
		}
		len1 = is->audio_buf_size - is->audio_buf_index;
		if (len1 > len)
			len1 = len;
		SDL_MixAudio(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len, SDL_MIX_MAXVOLUME);
		//memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
		len -= len1;
		stream += len1;
		is->audio_buf_index += len1;
	}
}

static Uint32 sdl_refresh_timer_cb(Uint32 interval, void *opaque) {
	SDL_Event event;
	event.type = FF_REFRESH_EVENT;
	event.user.data1 = opaque;
	SDL_PushEvent(&event);
	return 0; /* 0 means stop timer */
}

/* schedule a video refresh in 'delay' ms */
static void schedule_refresh(VideoState *is, int delay) {
	SDL_AddTimer(delay, sdl_refresh_timer_cb, is);
	//f << "current time: " << av_gettime()/1000000 << "\n";
}

void display_frame(VideoState *is)
{
	AVFrame *frame = av_frame_alloc();
	av_frame_ref(frame, disFrame.avFrame);
	
	sws_scale(is->sws_ctx, (uint8_t const* const*)frame->data,
		frame->linesize, 0, is->video_ctx->height, showFrame->data, showFrame->linesize);
	if (frame->linesize[0] >0 && frame->linesize[1] > 0 && frame->linesize[2] >0)
	{
		SDL_UpdateYUVTexture(texture, NULL, frame->data[0], frame->linesize[0],
			frame->data[1], frame->linesize[1],
			frame->data[2], frame->linesize[2]);
	}
	else if (frame->linesize[0] < 0 && frame->linesize[1] < 0 && frame->linesize[2] < 0) {
		SDL_UpdateYUVTexture(texture, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0],
			frame->data[1] + frame->linesize[1] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[1],
			frame->data[2] + frame->linesize[2] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[2]);
	}
	//SDL_UpdateTexture(texture, &showrect, frame->data[0], frame->linesize[0]);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, NULL, &showrect);
	SDL_RenderPresent(renderer);
	av_frame_unref(disFrame.avFrame);
	av_frame_unref(frame);
}

void video_display(VideoState *is) {

	SDL_Rect rect;
	VideoPicture *vp;
	float aspect_ratio;
	int w, h, x, y;
	int i;

	vp = &is->pictq[is->pictq_rindex];
	if (vp->bmp) {
		if (is->video_ctx->sample_aspect_ratio.num == 0) {
			aspect_ratio = 0;
		}
		else {
			aspect_ratio = av_q2d(is->video_ctx->sample_aspect_ratio) *
				is->video_ctx->width / is->video_ctx->height;
		}
		if (aspect_ratio <= 0.0) {
			aspect_ratio = (float)is->video_ctx->width /
				(float)is->video_ctx->height;
		}
#if 0
		h = screen->h;
		w = ((int)rint(h * aspect_ratio)) & -3;
		if (w > screen->w) {
			w = screen->w;
			h = ((int)rint(w / aspect_ratio)) & -3;
		}
		x = (screen->w - w) / 2;
		y = (screen->h - h) / 2;
#endif

		rect.x = 0;
		rect.y = 0;
		rect.w = is->video_ctx->width;
		rect.h = is->video_ctx->height;

#if 0
		SDL_LockMutex(screen_mutex);
		SDL_DisplayYUVOverlay(vp->bmp, &rect);
		SDL_UnlockMutex(screen_mutex);
#endif
		SDL_LockMutex(screen_mutex);
		SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, vp->bmp, NULL, NULL);
		SDL_RenderPresent(renderer);
	}
}

static int frame_queue_flush(QList<Frame> &listFrame)
{
	SDL_LockMutex(frame_mutex);
	while (!listFrame.isEmpty())
	{
		av_frame_unref(listFrame.front().avFrame);
		listFrame.pop_front();
	}
	SDL_UnlockMutex(frame_mutex);
	return 0;
}

static int frame_queue_get(VideoState *is, Frame &pFrame)
{
	if (listFrame.isEmpty())
	{
		return -1;
	}
	SDL_LockMutex(frame_mutex);
	
	av_frame_ref(pFrame.avFrame, listFrame.front().avFrame);
	//pFrame.avFrame = listFrame.front().avFrame;
	pFrame.pts = listFrame.front().pts;
	pFrame.num = listFrame.front().num;
	av_frame_unref(listFrame.front().avFrame);
	listFrame.pop_front();
	
	SDL_UnlockMutex(frame_mutex);

	return 0;
}

//video 刷新计时器
void video_refresh_timer(void *userdata) {

	VideoState *is = (VideoState *)userdata;
	VideoPicture *vp;
	double actual_delay, delay, sync_threshold, ref_clock, diff;

	if (is->video_st) {
		//if (is->pictq_size == 0) 
		if (bufferIsReady == 0)
		{
			schedule_refresh(is, 100);
		}

		if (listFrame.size() == 0)
		{
			//av_usleep(1000);
			schedule_refresh(is, 1);
		}
		else {
			//vp = &is->pictq[is->pictq_rindex];
			frame_queue_get(is, disFrame);
			//is->video_current_pts = vp->pts;
			is->video_current_pts = disFrame.pts;

			is->video_current_pts_time = av_gettime();
			//delay = vp->pts - is->frame_last_pts; /* the pts from last time */
			delay = disFrame.pts - is->frame_last_pts;
			if (delay <= 0 || delay >= 1.0) {
				/* if incorrect delay, use previous one */
				delay = is->frame_last_delay;
			}
			/* save for next time */
			is->frame_last_delay = delay;
			//is->frame_last_pts = vp->pts;
			is->frame_last_pts = disFrame.pts;


			/* update delay to sync to audio if not master source */
			if (is->av_sync_type != AV_SYNC_VIDEO_MASTER) {
				ref_clock = get_master_clock(is);
				//diff = vp->pts - ref_clock;
				diff = disFrame.pts - ref_clock;

				/* Skip or repeat the frame. Take delay into account
				FFPlay still doesn't "know if this is the best guess." */
				sync_threshold = (delay > AV_SYNC_THRESHOLD) ? delay : AV_SYNC_THRESHOLD;
				if (fabs(diff) < AV_NOSYNC_THRESHOLD) {
					if (diff <= -sync_threshold) {
						delay = 0;
					}
					else if (diff >= sync_threshold) {
						delay = 2 * delay;
					}
				}
			}
			is->frame_timer += delay;
			/* computer the REAL delay */
			actual_delay = is->frame_timer - (av_gettime() / 1000000.0);

			if (actual_delay < 0.010) {
				/* Really it should skip the picture instead */
				actual_delay = 0.010;
			}

			//f << "actual_delay: " << actual_delay << "\n";
			//av_usleep(actual_delay * 1000000);
			schedule_refresh(is, (int)(actual_delay * 1000 + 0.5));

			/* show the picture! */
			display_frame(is);
			//video_display(is);

			/* update queue for next picture! */
			/*if (++is->pictq_rindex == VIDEO_PICTURE_QUEUE_SIZE) {
			is->pictq_rindex = 0;
			}
			SDL_LockMutex(is->pictq_mutex);
			is->pictq_size--;
			SDL_CondSignal(is->pictq_cond);
			SDL_UnlockMutex(is->pictq_mutex);*/
		}
	}
	else {
		schedule_refresh(is, 100);
	}
}

void alloc_picture(void *userdata) {

	VideoState *is = (VideoState *)userdata;
	VideoPicture *vp;

	vp = &is->pictq[is->pictq_windex];
	if (vp->bmp) {
		// we already have one make another, bigger/smaller
		SDL_DestroyTexture(vp->bmp);
	}
	// Allocate a place to put our YUV image on that screen
	SDL_LockMutex(screen_mutex);
	/*vp->bmp = SDL_CreateYUVOverlay(is->video_ctx->width,
	is->video_ctx->height,
	SDL_YV12_OVERLAY,
	screen);*/
	vp->bmp = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING,
		is->video_ctx->width, is->video_ctx->height);
	SDL_UnlockMutex(screen_mutex);

	vp->width = is->video_ctx->width;
	vp->height = is->video_ctx->height;
	vp->allocated = 1;
}

void frame_queue_put(VideoState *is, AVFrame *pFrame, double pts)
{
	SDL_LockMutex(frame_mutex);
	Frame frame;
	av_frame_ref(frame.avFrame, pFrame);
	//frame.avFrame = pFrame;
	frame.pts = pts;
	frame.num = num_putin_list++;
	listFrame.push_back(frame);
	SDL_UnlockMutex(frame_mutex);
}

int queue_picture(VideoState *is, AVFrame *pFrame, double pts)
{
	VideoPicture *vp;
	int dst_pix_fmt;
	//use AVFrame instead
	//AVPicture pict;
	AVFrame *pict;
	pict = av_frame_alloc();

	/* wait until we have space for a new pic */
	SDL_LockMutex(is->pictq_mutex);
	while (is->pictq_size >= VIDEO_PICTURE_QUEUE_SIZE &&
		!is->quit) {
		SDL_CondWait(is->pictq_cond, is->pictq_mutex);
	}
	SDL_UnlockMutex(is->pictq_mutex);

	if (is->quit)
		return -1;

	// windex is set to 0 initially
	vp = &is->pictq[is->pictq_windex];

	/* allocate or resize the buffer! */
	if (!vp->bmp ||
		vp->width != is->video_ctx->width ||
		vp->height != is->video_ctx->height) {
		SDL_Event event;

		vp->allocated = 0;
		alloc_picture(is);
		if (is->quit) {
			return -1;
		}
	}
	/* We have a place to put our picture on the queue */
	//out_buffer = (unsigned char *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, is->video_ctx->width, is->video_ctx->height, 1));
	/*av_image_fill_arrays(pict->data, pict->linesize, out_buffer,
	AV_PIX_FMT_YUV420P, is->video_ctx->width, is->video_ctx->height, 1);*/


	if (vp->bmp) {
		uint8_t *pixels[4];
		int pitch[4];
		SDL_LockTexture(vp->bmp, nullptr, (void **)pixels, pitch);
		vp->pts = pts;

		dst_pix_fmt = AV_PIX_FMT_YUV420P;
		/* point pict at the queue */

#if 0
		pict.data[0] = vp->bmp->pixels[0];
		pict.data[1] = vp->bmp->pixels[2];
		pict.data[2] = vp->bmp->pixels[1];

		pict.linesize[0] = vp->bmp->pitches[0];
		pict.linesize[1] = vp->bmp->pitches[2];
		pict.linesize[2] = vp->bmp->pitches[1];
#endif

		// Convert the image into YUV format that SDL uses
#if 0
		sws_scale(is->sws_ctx, (uint8_t const * const *)pFrame->data,
			pFrame->linesize, 0, is->video_ctx->height,
			pict.data, pict.linesize);
#endif
		sws_scale(is->sws_ctx, (uint8_t const* const*)pFrame->data,
			pFrame->linesize, 0, is->video_ctx->height, pict->data, pict->linesize);

		//SDL_UpdateTexture(vp->bmp, NULL, pict->data[0], pict->linesize[0]);
		/*
		在这里直接播放可以显示正常,速度不对
		应将AVFrame存入队列中，待到要播放时，取出播放
		*/
		/*SDL_RenderClear(renderer);
		SDL_RenderCopy(renderer, vp->bmp, NULL, NULL);
		SDL_RenderPresent(renderer);*/


		SDL_UnlockTexture(vp->bmp);
		/* now we inform our display thread that we have a pic ready */
		if (++is->pictq_windex == VIDEO_PICTURE_QUEUE_SIZE) {
			is->pictq_windex = 0;
		}
		SDL_LockMutex(is->pictq_mutex);
		is->pictq_size++;
		SDL_UnlockMutex(is->pictq_mutex);
		av_frame_free(&pict);
	}
	return 0;
}

double synchronize_video(VideoState *is, AVFrame *src_frame, double pts) {

	double frame_delay;

	if (pts != 0) {
		/* if we have pts, set video clock to it */
		is->video_clock = pts;
	}
	else {
		/* if we aren't given a pts, set it to the clock */
		pts = is->video_clock;
	}
	/* update the video clock */
	frame_delay = av_q2d(is->video_ctx->time_base); //这是一个frame的标准delay(只显示一次)
													/* if we are repeating a frame, adjust clock accordingly */
	frame_delay += src_frame->repeat_pict * (frame_delay * 0.5);  //若会显示多次，则需要加上重复的次数乘以单次的标准delay时间，再乘以0.5(为何再乘以0.5，或者说除以2)
	is->video_clock += frame_delay;
	return pts;
}

int video_thread(void *arg) {
	int ret = -1;
	VideoState *is = (VideoState *)arg;
	AVPacket pkt1, *packet = &pkt1;
	int frameFinished;
	AVFrame *pFrame;
	double pts;
	AVRational tb = is->video_st->time_base;

	texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING,
		is->video_ctx->width, is->video_ctx->height);
	showrect.x = 0;
	showrect.y = 0;
	//showrect.w = is->video_ctx->width;
	//showrect.h = is->video_ctx->height;
	showrect.w = 640;
	showrect.h = 480;

	pFrame = av_frame_alloc();

	for (;;) {
		if (is->quit)
		{
			break;
		}
		if (listFrame.size() >= 5)
		{
			SDL_Delay(40);
			continue;
		}

		if (packet_queue_get(&is->videoq, packet, 1, is->quit) < 0) {
			// means we quit getting packets
			//f << "End of File:" << av_gettime() <<"\n";
			bufferIsReady = 0;
			continue;
			//break;
		}
		// if(packet_queue_get(&is->videoq, packet, 1) < 0) {
		//   // means we quit getting packets
		//   break;
		// }
		pts = 0;

		// Decode video frame
		//avcodec_decode_video2(is->video_ctx, pFrame, &frameFinished, packet);
		/*
		调用新接口解码
		*/
		ret = avcodec_send_packet(is->video_ctx, packet);
		if (ret < 0)
		{
			continue;
	}
		avcodec_receive_frame(is->video_ctx, pFrame);
		if (ret < 0)
		{
			continue;
		}

		/*if ((pts = av_frame_get_best_effort_timestamp(pFrame)) == AV_NOPTS_VALUE) {
		pts = av_frame_get_best_effort_timestamp(pFrame);
		}
		else {
		pts = 0;
		}*/

		//pts = (pFrame->pts == AV_NOPTS_VALUE) ? 0 : pFrame->pts * av_q2d(tb);
		pFrame->pts = pFrame->best_effort_timestamp;
		pts = av_q2d(is->video_st->time_base) * pFrame->pts;

		//pts *= av_q2d(is->video_st->time_base);
		pts = synchronize_video(is, pFrame, pts);
		
		frame_queue_put(is, pFrame, pts);
		//list_frame_get(is, disFrame);	//test
		av_frame_unref(pFrame);

		// Did we get a video frame?
#if 0
		if (frameFinished) {
			pts = synchronize_video(is, pFrame, pts);
			if (queue_picture(is, pFrame, pts) < 0) {
				break;
			}
		}
#endif
		av_packet_unref(packet);
}
	av_frame_unref(pFrame);
	return 0;
}

int stream_component_open(VideoState *is, int stream_index) {

	AVFormatContext *pFormatCtx = is->pFormatCtx;
	AVCodecContext *codecCtx = NULL;
	AVCodec *codec = NULL;
	SDL_AudioSpec wanted_spec, spec;

	if (stream_index < 0 || stream_index >= pFormatCtx->nb_streams) {
		return -1;
	}

	codec = avcodec_find_decoder(pFormatCtx->streams[stream_index]->codec->codec_id);
	if (!codec) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1;
	}

	codecCtx = avcodec_alloc_context3(codec);
	if (avcodec_copy_context(codecCtx, pFormatCtx->streams[stream_index]->codec) != 0) {
		fprintf(stderr, "Couldn't copy codec context");
		return -1; // Error copying codec context
	}


	if (codecCtx->codec_type == AVMEDIA_TYPE_AUDIO) {
		// Set audio settings from codec info
		wanted_spec.freq = 44100;
		wanted_spec.format = AUDIO_S16SYS;
		wanted_spec.channels = codecCtx->channels;
		wanted_spec.silence = 0;
		wanted_spec.samples = SDL_AUDIO_BUFFER_SIZE;
		wanted_spec.callback = audio_callback;
		wanted_spec.userdata = is;

		if (SDL_OpenAudio(&wanted_spec, &spec) < 0) {
			fprintf(stderr, "SDL_OpenAudio: %s\n", SDL_GetError());
			return -1;
		}
		is->audio_hw_buf_size = spec.size;
	}
	if (avcodec_open2(codecCtx, codec, NULL) < 0) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1;
	}

	switch (codecCtx->codec_type) {
	case AVMEDIA_TYPE_AUDIO:
		is->audioStream = stream_index;
		is->audio_st = pFormatCtx->streams[stream_index];
		is->audio_ctx = codecCtx;
		is->audio_buf_size = 0;
		is->audio_buf_index = 0;
		memset(&is->audio_pkt, 0, sizeof(is->audio_pkt));
		packet_queue_init(&is->audioq);
		SDL_PauseAudio(0);
		break;
	case AVMEDIA_TYPE_VIDEO:
		is->videoStream = stream_index;
		is->video_st = pFormatCtx->streams[stream_index];
		is->video_ctx = codecCtx;

		is->frame_timer = (double)av_gettime() / 1000000.0;
		is->frame_last_delay = 40e-3;
		is->video_current_pts_time = av_gettime();

		/*out_buffer = (unsigned char *)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, is->video_ctx->width, is->video_ctx->height, 1));
		av_image_fill_arrays(showFrame->data, showFrame->linesize, out_buffer,
		AV_PIX_FMT_YUV420P, is->video_ctx->width, is->video_ctx->height, 1);*/

		packet_queue_init(&is->videoq);
		is->video_tid = SDL_CreateThread(video_thread, "video_thread", is);
		is->sws_ctx = sws_getContext(is->video_ctx->width, is->video_ctx->height,
			is->video_ctx->pix_fmt, 640,
			480, AV_PIX_FMT_YUV420P,
			SWS_BICUBIC, NULL, NULL, NULL);
		break;
	default:
		break;
	}
}

int decode_thread(void *arg) {

	VideoState *is = (VideoState *)arg;
	AVFormatContext *pFormatCtx = NULL;
	AVPacket pkt1, *packet = &pkt1;

	int video_index = -1;
	int audio_index = -1;
	int i;

	is->videoStream = -1;
	is->audioStream = -1;

	//global_video_state = is;

	// Open video file
	if (avformat_open_input(&pFormatCtx, is->filename, NULL, NULL) != 0)
		return -1; // Couldn't open file

	is->pFormatCtx = pFormatCtx;

	// Retrieve stream information
	if (avformat_find_stream_info(pFormatCtx, NULL)<0)
		return -1; // Couldn't find stream information

				   // Dump information about file onto standard error
	av_dump_format(pFormatCtx, 0, is->filename, 0);

	// Find the first video stream

	for (i = 0; i<pFormatCtx->nb_streams; i++) {
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO &&
			video_index < 0) {
			video_index = i;
		}
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_AUDIO &&
			audio_index < 0) {
			audio_index = i;
		}
	}
	if (audio_index >= 0) {
		stream_component_open(is, audio_index);
	}
	if (video_index >= 0) {
		stream_component_open(is, video_index);
	}

#if 0
	if (is->videoStream < 0 || is->audioStream < 0) {
		fprintf(stderr, "%s: could not open codecs\n", is->filename);
		goto fail;
	}
#endif

	// main decode loop
	for (;;) {
		if (is->quit) {
			break;
		}
		// seek stuff goes here
		if (is->seek_req) {
			int stream_index = -1;
			int64_t seek_target = is->seek_pos;

			if (is->videoStream >= 0) stream_index = is->videoStream;
			else if (is->audioStream >= 0) stream_index = is->audioStream;

			int duration = pFormatCtx->duration;
			AVRational xx = pFormatCtx->streams[stream_index]->time_base;
			if (stream_index >= 0) {
				AVRational temp = AVRational{ 1, AV_TIME_BASE };
				/*seek_target = av_rescale_q(seek_target, temp,
					pFormatCtx->streams[stream_index]->time_base);*/

				seek_target = seek_target * (temp.num * pFormatCtx->streams[stream_index]->time_base.den / (double)(temp.den * pFormatCtx->streams[stream_index]->time_base.num));
			}
			int ret = av_seek_frame(is->pFormatCtx, stream_index,
				seek_target, is->seek_flags);
			if (ret < 0) {
				fprintf(stderr, "%s: error while seeking\n",
					is->pFormatCtx->filename);
			}
			else {

				if (is->audioStream >= 0) {
					packet_queue_flush(&is->audioq);
					packet_queue_put(&is->audioq, &flush_pkt);
				}
				if (is->videoStream >= 0) {
					packet_queue_flush(&is->videoq);
					packet_queue_put(&is->videoq, &flush_pkt);
				}
			}
			is->seek_req = 0;
		}

		if (is->audioq.size > MAX_AUDIOQ_SIZE ||
			is->videoq.size > MAX_VIDEOQ_SIZE) {
			SDL_Delay(10);
			continue;
			}
		if (av_read_frame(is->pFormatCtx, packet) < 0) {
			//bufferIsReady = 0;
			if (is->pFormatCtx->pb->error == 0) {
				SDL_Delay(100); /* no error; wait for user input */
				continue;
			}
			if (is->pFormatCtx->pb->error == AVERROR_EOF)
			{
				eof = 1;
			}
			else {
				break;
			}
		}

		// Is this a packet from the video stream?
		if (packet->stream_index == is->videoStream) {
			packet_queue_put(&is->videoq, packet);

			if (is->videoq.nb_packets >= 50)
			{
				bufferIsReady = 1;
			}

			/*计算实时帧率fps*/
#if 0
			if (num_decoded++ == 50)
			{
				num_decoded = 0;
				/*QTime temp = QTime::currentTime();
				double s = qtime.msecsTo(temp);
				if (s != 0)
				{
					double fps = double(50 * 1000 / s);
				}
				qtime = temp;*/
			}
#endif
		}
		else if (packet->stream_index == is->audioStream) {
			packet_queue_put(&is->audioq, packet);
		}
		else {
			av_free_packet(packet);
		}
		}
	/* all done - wait for it */
	while (!is->quit) {
		SDL_Delay(100);
	}

fail:
	if (1) {
		SDL_Event event;
		event.type = FF_QUIT_EVENT;
		event.user.data1 = is;
		SDL_PushEvent(&event);
	}
	return 0;
}

void stream_seek(VideoState *is, int64_t pos, int rel) {

	if (!is->seek_req) {
		is->seek_pos = pos;
		is->seek_flags = rel < 0 ? AVSEEK_FLAG_BACKWARD : 0;
		//is->seek_flags = AVSEEK_FLAG_ANY;
		is->seek_req = 1;
	}
}

LXPlayerCore::LXPlayerCore(QObject *parent)
	: QObject(parent), m_is(NULL)
{
}

LXPlayerCore::~LXPlayerCore()
{
	SDL_WaitThread(m_is->parse_tid, NULL);
	SDL_WaitThread(m_is->video_tid, NULL);
		
	packet_queue_abort(&m_is->audioq);
	packet_queue_abort(&m_is->videoq);

	packet_queue_flush(&m_is->audioq);
	packet_queue_flush(&m_is->videoq);

	//avcodec_close(m_is->audio_ctx);
	//avcodec_close(m_is->video_ctx);
	avcodec_free_context(&m_is->audio_ctx);
	avcodec_free_context(&m_is->video_ctx);

	avformat_close_input(&m_is->pFormatCtx);
	//avformat_free_context(m_is->pFormatCtx);
	SDL_DestroyCond(m_is->audioq.cond);
	SDL_DestroyMutex(m_is->audioq.mutex);

	SDL_DestroyCond(m_is->videoq.cond);
	SDL_DestroyMutex(m_is->videoq.mutex);

	SDL_DestroyCond(m_is->pictq_cond);
	SDL_DestroyMutex(m_is->pictq_mutex);

	SDL_DestroyMutex(frame_mutex);
	SDL_DestroyMutex(screen_mutex);

	sws_freeContext(m_is->sws_ctx);

	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(screen);

	av_lockmgr_register(NULL);
	avformat_network_deinit();

	av_free(m_is);

	SDL_Quit();
}

void LXPlayerCore::run()
{
	SDL_Event       event;
	frame_mutex = SDL_CreateMutex();
	//VideoState      *is;
	
	m_is = (VideoState *)av_mallocz(sizeof(VideoState));
	showFrame = av_frame_alloc();

	// Register all formats and codecs
	av_register_all();
	avformat_network_init();

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		fprintf(stderr, "Could not initialize SDL - %s\n", SDL_GetError());
		exit(1);
	}

	//f << "play start:" << av_gettime() << "\n";
	// Make a screen to put our video
	screen = SDL_CreateWindowFrom(m_hwnd);
	//screen = SDL_CreateWindow("", 0, 0, 640, 480, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
	renderer = SDL_CreateRenderer(screen, -1, 0);

	if (!screen) {
		fprintf(stderr, "SDL: could not set video mode - exiting\n");
		exit(1);
	}

	screen_mutex = SDL_CreateMutex();

	//av_strlcpy(is->filename, argv[1], sizeof(is->filename));
	//将文件名复制给is->filename
	strcpy(m_is->filename, m_url.c_str());

	m_is->pictq_mutex = SDL_CreateMutex();
	m_is->pictq_cond = SDL_CreateCond();

	schedule_refresh(m_is, 40);

	m_is->av_sync_type = DEFAULT_AV_SYNC_TYPE;
	m_is->parse_tid = SDL_CreateThread(decode_thread, "decode_thread", m_is);
	if (!m_is->parse_tid) {
		av_free(m_is);
		return ;
	}

	av_init_packet(&flush_pkt);
	flush_pkt.data = (uint8_t *)"FLUSH";

	for (;;) {
		double incr, pos;
		//refresh_loop_wait_event(is, &event);
		SDL_WaitEvent(&event);
		switch (event.type) {
		case SDL_KEYDOWN:
			switch (event.key.keysym.sym) {
			case SDLK_LEFT:
				incr = -10.0;
				goto do_seek;
			case SDLK_RIGHT:
				incr = 10.0;
				goto do_seek;
			case SDLK_UP:
				incr = 60.0;
				goto do_seek;
			case SDLK_DOWN:
				incr = -60.0;
				goto do_seek;
			do_seek:
				if (m_is) {
					pos = get_master_clock(m_is);
					pos += incr;
					stream_seek(m_is, (int64_t)(pos * AV_TIME_BASE), incr);
				}
				break;
			default:
				break;
			}
			break;
		case FF_QUIT_EVENT:
		case SDL_QUIT:
			m_is->quit = 1;
			/*
			* If the video has finished playing, then both the picture and
			* audio queues are waiting for more data.  Make them stop
			* waiting and terminate normally.
			*/
			SDL_CondSignal(m_is->audioq.cond);
			SDL_CondSignal(m_is->videoq.cond);
			//SDL_Quit();
			return ;
			break;
		case FF_REFRESH_EVENT:
			video_refresh_timer(event.user.data1);
			break;
		default:
			break;
		}
	}
	return ;

}

void LXPlayerCore::setUrl(QString url)
{
	m_url = url.toStdString();
}

void LXPlayerCore::stop()
{
	m_is->quit = 1;
}


void LXPlayerCore::forward()
{
	seek(10);
}


void LXPlayerCore::backward()
{
	seek(-10);
}

void LXPlayerCore::setHWND(HWND h)
{
	m_hwnd = h;
}

void LXPlayerCore::seek(double incr)
{
	double pos = get_master_clock(m_is);
	pos += incr;
	stream_seek(m_is, (int64_t)(pos * AV_TIME_BASE), incr);
}
