#include "SimplePlayer.h"

SimplePlayer::SimplePlayer(QWidget *parent)
	: QMainWindow(parent), widget(NULL)
{
	ui.setupUi(this);

	playBtn = new QPushButton("play");
	stopBtn = new QPushButton("stop");
	urlEdit = new QLineEdit;
	m_forward = new QPushButton(QStringLiteral("快进"));
	m_backward = new QPushButton(QStringLiteral("快退"));
	//urlEdit->setText(QString("rtmp://live.hkstv.hk.lxdns.com/live/hks"));
	urlComboBox = new QComboBox;
	urlComboBox->addItem(QString("rtmp://live.hkstv.hk.lxdns.com/live/hks"));
	urlComboBox->addItem(QString("D:/learnproject/videoSource/numb.mp4"));
	urlComboBox->addItem("D:/learnproject/videoSource/ItsMyLife.mp3");
	QHBoxLayout *hlayout = new QHBoxLayout;

	ui.centralWidget->setLayout(hlayout);
	hlayout->addWidget(playBtn);
	hlayout->addWidget(stopBtn);
	hlayout->addWidget(urlEdit);
	
	hlayout->addWidget(m_forward);
	hlayout->addWidget(m_backward);
	hlayout->addWidget(urlComboBox);
	//audio = new Audio;
	
	//m_corePlay = new LXPlayerCore(this);

	m_threadPool = new QThreadPool;

	connect(playBtn, &QPushButton::clicked, this, &SimplePlayer::slotPlay);
	connect(stopBtn, &QPushButton::clicked, this, &SimplePlayer::slotStop);
	//connect(play_thread, &VideoPlayer_Thread::sig_GetOneFrame, this, &SimplePlayer::slotGetOneFrame);
	connect(m_forward, &QPushButton::clicked, this, &SimplePlayer::slotForward);
	connect(m_backward, &QPushButton::clicked, this, &SimplePlayer::slotBackward);

	
}

void SimplePlayer::setH(HWND h)
{
#if TP
play_thread->setHWND(h);
#endif

#if COREPLAY
	m_corePlay->setHWND(h);
#endif
}

void SimplePlayer::slotPlay()
{
	//QString url = urlEdit->text();

	//if (m_corePlay!=nullptr)
	//{
	//	m_corePlay->stop();
	//}

	QString url;
	if (!urlEdit->text().isEmpty())
	{
		 url= urlEdit->text();
	}
	else
		url = urlComboBox->currentText();

#if TP
	if (play_thread == NULL)
	{
		int x = 0;
	}

	play_thread = new VideoPlayer_Thread;
	play_thread->setFileName(url);
	//
	play_thread->setHWND(h);
	play_thread->start();
	//m_threadPool->start(play_thread);
#endif

#if COREPLAY
	if (widget == NULL)
	{
		widget = new QWidget;
		widget->show();
		h = (HWND)widget->winId();
	}
	if (m_corePlay == NULL)
	{
		m_corePlay = new LXPlayerCore(this);
		connect(m_corePlay, &LXPlayerCore::signalFinished, this, &SimplePlayer::slotStop);
		m_corePlay->setUrl(url);

		m_corePlay->setHWND(h);
		m_corePlay->start();
	}
	
	//m_threadPool->start(m_corePlay);
#endif
	//m_threadPool->start(m_corePlay);
	//play_thread->setFileName(url);
	//play_thread->start();
	//audio->PlayAudio(url.toStdString());
	
}

void SimplePlayer::slotStop()
{
#if TP
	if (play_thread != NULL)
	{
		play_thread->stop(true);
	}
#endif

#if COREPLAY
	if (m_corePlay != NULL)
	{
		m_corePlay->stop();
		m_corePlay->wait();
		delete m_corePlay;
		m_corePlay = NULL;
	}
	if (widget != NULL)
	{
		delete widget;
		widget = NULL;
	}
#endif
	//m_q->stop();
}

void SimplePlayer::slotGetOneFrame(QImage img)
{
	mImage = img;
	update();
}

void SimplePlayer::slotForward()
{
	m_tenSecs += 10 * 1000000;
	//play_thread->seek(m_tenSecs);
	m_corePlay->forward();

}

void SimplePlayer::slotBackward()
{
	int backTenSecs = -10 * 1000000;
//	play_thread->seek(backTenSecs);
	m_corePlay->backward();
}

void SimplePlayer::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);

	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::TextAntialiasing);
	painter.setRenderHint(QPainter::SmoothPixmapTransform);
	painter.setRenderHint(QPainter::HighQualityAntialiasing);

	painter.setBrush(Qt::black);
	painter.drawRect(0, 0, this->width(), this->height()); //先画成黑色


	if (mImage.size().width() <= 0) return;

	///将图像按比例缩放成和窗口一样大小
	QImage img = mImage.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation); //这里效率比较低下  还不知道如何优化

	int x = this->width() - img.width();
	int y = this->height() - img.height();

	x /= 2;
	y /= 2;

	painter.drawImage(QPoint(x, y), img); //画出图像
}
