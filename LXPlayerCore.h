#pragma once

#include <QObject>
#include <QList>
#include <QRunnable>
#include <QThread>
#include <ctime>
#include <cassert>
#include <windows.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libswresample/swresample.h>
#include <libavutil/time.h>
#include <libavutil/imgutils.h>

#include <SDL.h>
#include <SDL_main.h>
#include <SDL_thread.h>
}

#define SDL_AUDIO_BUFFER_SIZE 1024
#define MAX_AUDIO_FRAME_SIZE 192000

#define MAX_AUDIOQ_SIZE (5 * 16 * 1024)
#define MAX_VIDEOQ_SIZE (5 * 256 * 1024)

#define AV_SYNC_THRESHOLD 0.01
#define AV_NOSYNC_THRESHOLD 10.0

#define SAMPLE_CORRECTION_PERCENT_MAX 10
#define AUDIO_DIFF_AVG_NB 20

#define FF_REFRESH_EVENT (SDL_USEREVENT)
#define FF_QUIT_EVENT (SDL_USEREVENT + 1)

#define VIDEO_PICTURE_QUEUE_SIZE 1

#define DEFAULT_AV_SYNC_TYPE AV_SYNC_VIDEO_MASTER
#define AV_TIME_BASE_Q   (AVRational){1, AV_TIME_BASE}

enum {
	AV_SYNC_AUDIO_MASTER,
	AV_SYNC_VIDEO_MASTER,
	AV_SYNC_EXTERNAL_MASTER
};

typedef struct PacketQueue {
	AVPacketList *first_pkt, *last_pkt;
	int nb_packets;
	int size;
	SDL_mutex *mutex;
	SDL_cond *cond;
	int abort_request;
} PacketQueue;

typedef struct Frame
{
	Frame::Frame()
	{
		avFrame = av_frame_alloc();
	}
	AVFrame *avFrame;
	double pts;
	int num;
} Frame;

typedef struct VideoPicture {
	SDL_Texture *bmp;
	int width, height; /* source height & width */
	int allocated;
	double pts;
} VideoPicture;

typedef struct VideoState {

	AVFormatContext *pFormatCtx;
	int             videoStream, audioStream;

	int             av_sync_type;
	double          external_clock; /* external clock base */
	int64_t         external_clock_time;
	int             seek_req;
	int             seek_flags;
	int64_t         seek_pos;

	double          audio_clock;
	AVStream        *audio_st;
	AVCodecContext  *audio_ctx;
	PacketQueue     audioq;
	uint8_t         audio_buf[(MAX_AUDIO_FRAME_SIZE * 3) / 2];
	unsigned int    audio_buf_size;
	unsigned int    audio_buf_index;
	AVFrame         audio_frame;
	AVPacket        audio_pkt;
	uint8_t         *audio_pkt_data;
	int             audio_pkt_size;
	int             audio_hw_buf_size;
	double          audio_diff_cum; /* used for AV difference average computation */
	double          audio_diff_avg_coef;
	double          audio_diff_threshold;
	int             audio_diff_avg_count;
	double          frame_timer;
	double          frame_last_pts;
	double          frame_last_delay;
	double          video_clock; ///<pts of last decoded frame / predicted pts of next decoded frame
	double          video_current_pts; ///<current displayed pts (different from video_clock if frame fifos are used)
	int64_t         video_current_pts_time;  ///<time (av_gettime) at which we updated video_current_pts - used to have running video pts
	AVStream        *video_st;
	AVCodecContext  *video_ctx;
	PacketQueue     videoq;
	struct SwsContext *sws_ctx;

	VideoPicture    pictq[VIDEO_PICTURE_QUEUE_SIZE];
	//Frame			disFrame;
	//std::list<Frame> listFrame;

	int             pictq_size, pictq_rindex, pictq_windex;
	SDL_mutex       *pictq_mutex;
	SDL_cond        *pictq_cond;

	SDL_Thread      *parse_tid;
	SDL_Thread      *video_tid;

	char            filename[1024];
	int             quit;

} VideoState;

class LXPlayerCore : public QObject, public QThread
{
	Q_OBJECT

public:
	LXPlayerCore(QObject *parent);
	~LXPlayerCore();

	void run();
	void setUrl(QString url);
	void stop();
	void forward();
	void backward();
	void setHWND(HWND h);

signals:
	void signalFinished();

private:
	void seek(double incr);
	VideoState *m_is;
	std::string m_url;
	HWND m_hwnd;
};
