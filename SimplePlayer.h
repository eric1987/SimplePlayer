#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_SimplePlayer.h"
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QPainter>
#include <QComboBox>
#include <QThreadPool>
//#include "videoplayer_thread.h"
#include "LXPlayerCore.h"
//#include "Audio.h"
#include "QtClass.h"

#define COREPLAY 1
#define TP 0

class SimplePlayer : public QMainWindow
{
	Q_OBJECT

public:
	SimplePlayer(QWidget *parent = Q_NULLPTR);

	void setH(HWND h);
public slots:
	void slotPlay();
	void slotStop();
	void slotGetOneFrame(QImage);
	void slotForward();
	void slotBackward();

protected:
	void paintEvent(QPaintEvent *event);

private:
	Ui::SimplePlayerClass ui;
	QPushButton *playBtn;
	QPushButton *stopBtn;
	QPushButton *m_forward;
	QPushButton *m_backward;
	QLineEdit *urlEdit;
	QComboBox *urlComboBox;

#if TP
	VideoPlayer_Thread *play_thread;
#endif
	
	
#if COREPLAY
	LXPlayerCore *m_corePlay;
#endif
	
	QImage mImage;

	int m_tenSecs = 0;
	QThreadPool *m_threadPool;
	QWidget *widget;
	HWND h;
	QtClass *m_q;

	//Audio *audio;
};
