#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QVector>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>
#include <QVBoxLayout>
#include <QToolTip>
#include "zoomablechartview.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void readSerialData();
    void scanAndConnect();
    void saveMinuteAverage();
    void setupCharts();
    void loadCsvHistory(const QString &filePath);
    void showPointTooltip(const QPointF &point, bool state);

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QTimer *portScanTimer;

    QFile csvFile;
    QTextStream csvStream;
    QTimer *minuteTimer;

    QVector<double> tempBuffer;
    QVector<double> humBuffer;
    QVector<double> pressBuffer;
    QVector<double> micBuffer;
    QVector<double> aqiBuffer;
    QVector<double> tvocBuffer;
    QVector<double> eco2Buffer;

    QLineSeries *seriesLive[7];
    QScatterSeries *seriesHistory[7];
    ZoomableChartView *chartViewLive[7];
    ZoomableChartView *chartViewHistory[7];
};

#endif // MAINWINDOW_H
