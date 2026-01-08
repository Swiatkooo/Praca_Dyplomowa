#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("Aplikacja Pomiarowa");
    setWindowFlags(windowFlags() & ~Qt::WindowMaximizeButtonHint);

    ui->lineEditTemp->setReadOnly(true);
    ui->lineEditHum->setReadOnly(true);
    ui->lineEditPress->setReadOnly(true);
    ui->lineEditMic->setReadOnly(true);
    ui->lineEditAqi->setReadOnly(true);
    ui->lineEditTvoc->setReadOnly(true);
    ui->lineEditEco2->setReadOnly(true);

    serial = new QSerialPort(this);
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readSerialData);

    connect(serial, &QSerialPort::errorOccurred, this,
            [this](QSerialPort::SerialPortError error){
                if (error == QSerialPort::ResourceError) {

                    ui->lineEditTemp->clear();
                    ui->lineEditHum->clear();
                    ui->lineEditPress->clear();
                    ui->lineEditMic->clear();
                    ui->lineEditAqi->clear();
                    ui->lineEditTvoc->clear();
                    ui->lineEditEco2->clear();

                    ui->statusbar->showMessage("Połączenie utracone, oczekiwanie...");
                    serial->close();
                }
            });

    portScanTimer = new QTimer(this);
    connect(portScanTimer, &QTimer::timeout, this, &MainWindow::scanAndConnect);
    portScanTimer->start(2000);

    csvFile.setFileName("data.csv");
    if (csvFile.open(QIODevice::Append | QIODevice::Text))
        csvStream.setDevice(&csvFile);

    if (csvFile.size() == 0) {
        csvStream << "Timestamp,Temp,Hum,Press,Mic,AQI,TVOC,eCO2\n";
    }

    minuteTimer = new QTimer(this);
    connect(minuteTimer, &QTimer::timeout, this, &MainWindow::saveMinuteAverage);
    minuteTimer->start(60000); // 60 sekund

    setupCharts();
    loadCsvHistory("data.csv");
}



MainWindow::~MainWindow()
{
    if(serial->isOpen())
        serial->close();
    delete ui;
}

void MainWindow::scanAndConnect()
{
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    if (serial->isOpen()) {
        bool stillExists = false;
        for (const QSerialPortInfo &info : ports) {
            if (info.portName() == serial->portName()) {
                stillExists = true;
                break;
            }
        }
        if (!stillExists) {
            serial->close();
            ui->statusbar->showMessage("Urządzenie odłączone, oczekiwanie...");
        }
        return;
    }

    for (const QSerialPortInfo &info : ports) {
        if (info.description().contains("STLink Virtual COM Port")) {
            serial->setPort(info);
            serial->setBaudRate(QSerialPort::Baud115200);
            serial->setDataBits(QSerialPort::Data8);
            serial->setParity(QSerialPort::NoParity);
            serial->setStopBits(QSerialPort::OneStop);
            serial->setFlowControl(QSerialPort::NoFlowControl);

            if (serial->open(QIODevice::ReadOnly)) {
                ui->statusbar->showMessage("Połączono z " + serial->portName());
                return;
            }
        }
    }

    ui->statusbar->showMessage("Oczekiwanie na urządzenie...");
}

void MainWindow::saveMinuteAverage()
{
    if (tempBuffer.isEmpty()) return;

    auto avg = [](const QVector<double> &buf) {
        return std::accumulate(buf.begin(), buf.end(), 0.0) / buf.size();
    };

    double avgTemp  = avg(tempBuffer);
    double avgHum   = avg(humBuffer);
    double avgPress = avg(pressBuffer);
    double avgMic   = avg(micBuffer);
    int    avgAqi   = qRound(avg(aqiBuffer));
    double avgTvoc  = avg(tvocBuffer);
    double avgEco2  = avg(eco2Buffer);

    QDateTime now = QDateTime::currentDateTime();
    qint64 t = now.toMSecsSinceEpoch();

    csvStream << now.toString("yyyy-MM-dd hh:mm:ss") << ","
              << avgTemp << "," << avgHum << "," << avgPress << ","
              << avgMic << "," << avgAqi << "," << avgTvoc << "," << avgEco2 << "\n";
    csvStream.flush();

    seriesHistory[0]->append(t, avgTemp);
    seriesHistory[1]->append(t, avgHum);
    seriesHistory[2]->append(t, avgPress);
    seriesHistory[3]->append(t, avgMic);
    seriesHistory[4]->append(t, avgAqi);
    seriesHistory[5]->append(t, avgTvoc);
    seriesHistory[6]->append(t, avgEco2);

    tempBuffer.clear();
    humBuffer.clear();
    pressBuffer.clear();
    micBuffer.clear();
    aqiBuffer.clear();
    tvocBuffer.clear();
    eco2Buffer.clear();
}


void MainWindow::loadCsvHistory(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream in(&file);
    bool skipHeader = true;

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (skipHeader) { skipHeader = false; continue; }

        QStringList parts = line.split(",");
        if (parts.size() != 8) continue;

        QDateTime timestamp = QDateTime::fromString(parts[0], "yyyy-MM-dd hh:mm:ss");
        if (!timestamp.isValid()) continue;

        bool ok[7];
        double values[7];
        for (int i = 0; i < 7; ++i)
            values[i] = parts[i + 1].toDouble(&ok[i]);

        if (std::all_of(std::begin(ok), std::end(ok), [](bool b){ return b; })) {
            qint64 t = timestamp.toMSecsSinceEpoch();
            for (int i = 0; i < 7; ++i)
                seriesHistory[i]->append(t, values[i]);
        }
    }

    for (int i = 0; i < 7; ++i) {
        auto *chart = chartViewHistory[i]->chart();
        auto *axisX = qobject_cast<QDateTimeAxis*>(chart->axes(Qt::Horizontal).first());
        auto *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());

        if (axisX && !seriesHistory[i]->points().isEmpty()) {
            qint64 minT = seriesHistory[i]->points().first().x();
            qint64 maxT = seriesHistory[i]->points().last().x();
            axisX->setMin(QDateTime::fromMSecsSinceEpoch(minT));
            axisX->setMax(QDateTime::fromMSecsSinceEpoch(maxT));
        }
        if (axisY && !seriesHistory[i]->points().isEmpty()) {
            switch (i) {
            case 0: axisY->setRange(0, 60); break;   // temperatura
            case 1: axisY->setRange(0, 100); break;  // wilgotność
            case 2: axisY->setRange(900, 1100); break; // ciśnienie
            case 3: axisY->setRange(20, 100); break; // hałas
            case 4: axisY->setRange(0, 5); break;  // AQI
            case 5: axisY->setRange(0, 2000); break; // TVOC
            case 6: axisY->setRange(400, 2000); break; // eCO₂
            }
        }
    }



    file.close();
}

void MainWindow::setupCharts()
{
    QString titles[7] = {
        "Temperatura [°C]", "Wilgotność [%]", "Ciśnienie [hPa]",
        "Hałas [dB SPL]", "AQI", "TVOC [ppb]", "eCO₂ [ppm]"
    };

    QWidget *liveWidgets[7] = {
        ui->widget_100, ui->widget_102, ui->widget_104,
        ui->widget_106, ui->widget_108, ui->widget_110, ui->widget_112
    };

    QWidget *historyWidgets[7] = {
        ui->widget_101, ui->widget_103, ui->widget_105,
        ui->widget_107, ui->widget_109, ui->widget_111, ui->widget_113
    };

    for (int i = 0; i < 7; ++i) {

        seriesLive[i] = new QLineSeries();
        seriesLive[i]->setName("Sesja bieżąca");
        QPen pen(Qt::blue);
        pen.setWidth(2);
        seriesLive[i]->setPen(pen);

        seriesHistory[i] = new QScatterSeries();
        seriesHistory[i]->setName("Historia + bieżące");
        seriesHistory[i]->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        seriesHistory[i]->setMarkerSize(6);
        seriesHistory[i]->setColor(Qt::blue);
        seriesHistory[i]->setBorderColor(Qt::blue);
        seriesHistory[i]->setBrush(Qt::blue);

        QDateTimeAxis *axisXLive = new QDateTimeAxis;
        axisXLive->setTitleText("Czas");

        QValueAxis *axisYLive = new QValueAxis;
        axisYLive->setTitleText(titles[i]);

        QDateTimeAxis *axisXHist = new QDateTimeAxis;
        axisXHist->setTitleText("Czas");

        QValueAxis *axisYHist = new QValueAxis;
        axisYHist->setTitleText(titles[i]);

        connect(seriesHistory[i], &QScatterSeries::hovered, this, &MainWindow::showPointTooltip);
        connect(seriesLive[i], &QLineSeries::hovered, this, &MainWindow::showPointTooltip);

        if (i == 0) {
            // sesja bieżąca
            axisXLive->setFormat("hh:mm:ss");
        } else {
            // historia
            axisXHist->setFormat("dd.MM hh:mm");
        }

        QChart *chartLive = new QChart();
        chartLive->addAxis(axisXLive, Qt::AlignBottom);
        chartLive->addAxis(axisYLive, Qt::AlignLeft);
        chartLive->addSeries(seriesLive[i]);
        seriesLive[i]->attachAxis(axisXLive);
        seriesLive[i]->attachAxis(axisYLive);
        chartLive->setTitle(titles[i] + " – sesja");

        chartViewLive[i] = new ZoomableChartView(chartLive);
        chartViewLive[i]->setRenderHint(QPainter::Antialiasing);
        chartViewLive[i]->setMouseTracking(true);
        chartViewLive[i]->setInteractive(true);

        QChart *chartHistory = new QChart();
        chartHistory->addAxis(axisXHist, Qt::AlignBottom);
        chartHistory->addAxis(axisYHist, Qt::AlignLeft);
        chartHistory->addSeries(seriesHistory[i]);
        seriesHistory[i]->attachAxis(axisXHist);
        seriesHistory[i]->attachAxis(axisYHist);
        chartHistory->setTitle(titles[i] + " – historia");

        chartViewHistory[i] = new ZoomableChartView(chartHistory);
        chartViewHistory[i]->setRenderHint(QPainter::Antialiasing);
        chartViewHistory[i]->setMouseTracking(true);
        chartViewHistory[i]->setInteractive(true);

        auto *layoutLive = new QVBoxLayout(liveWidgets[i]);
        layoutLive->setContentsMargins(0,0,0,0);
        layoutLive->addWidget(chartViewLive[i]);

        auto *layoutHistory = new QVBoxLayout(historyWidgets[i]);
        layoutHistory->setContentsMargins(0,0,0,0);
        layoutHistory->addWidget(chartViewHistory[i]);
    }
}

void MainWindow::showPointTooltip(const QPointF &point, bool state)
{
    if (!state) {
        QToolTip::hideText();
        return;
    }

    QString timeString = QDateTime::fromMSecsSinceEpoch(point.x())
                             .toString("dd.MM.yyyy hh:mm:ss");

    QString text = QString("%1\nWartość: %2")
                       .arg(timeString)
                       .arg(point.y());

    QToolTip::showText(QCursor::pos(), text);
}



void MainWindow::readSerialData()
{
    while (serial->canReadLine()) {
        QByteArray line = serial->readLine().trimmed();
        QString str = QString::fromUtf8(line);

        QRegularExpression rx(
            R"(T=([0-9\.]+)C\s+H=([0-9\.]+)%\s+P=([0-9\.]+)hPa\s+MIC=([0-9\.]+)dB SPL\s+AQI=(\d+)\s+TVOC=(\d+)ppb\s+eCO2=(\d+)ppm\s+Status=0x([0-9A-Fa-f]+))"
            );
        QRegularExpressionMatch match = rx.match(str);

        if (match.hasMatch()) {
            double tempVal = match.captured(1).toDouble();
            double humVal  = match.captured(2).toDouble();
            double pressVal= match.captured(3).toDouble();
            double micVal  = match.captured(4).toDouble();
            double aqiVal  = match.captured(5).toDouble();
            double tvocVal = match.captured(6).toDouble();
            double eco2Val = match.captured(7).toDouble();

            // aktualizacja UI
            ui->lineEditTemp->setText(QString::number(tempVal) + " °C");
            ui->lineEditHum->setText(QString::number(humVal) + " %");
            ui->lineEditPress->setText(QString::number(pressVal) + " hPa");
            ui->lineEditMic->setText(QString::number(micVal) + " dB SPL");
            ui->lineEditAqi->setText(QString::number(aqiVal));
            ui->lineEditTvoc->setText(QString::number(tvocVal) + " ppb");
            ui->lineEditEco2->setText(QString::number(eco2Val) + " ppm");

            tempBuffer.append(tempVal);
            humBuffer.append(humVal);
            pressBuffer.append(pressVal);
            micBuffer.append(micVal);
            aqiBuffer.append(aqiVal);
            tvocBuffer.append(tvocVal);
            eco2Buffer.append(eco2Val);

            QDateTime now = QDateTime::currentDateTime();
            qint64 t = now.toMSecsSinceEpoch();

            seriesLive[0]->append(t, tempVal);
            seriesLive[1]->append(t, humVal);
            seriesLive[2]->append(t, pressVal);
            seriesLive[3]->append(t, micVal);
            seriesLive[4]->append(t, aqiVal);
            seriesLive[5]->append(t, tvocVal);
            seriesLive[6]->append(t, eco2Val);

            for (int i = 0; i < 7; ++i) {
                auto *chart = chartViewLive[i]->chart();
                auto *axisX = qobject_cast<QDateTimeAxis*>(chart->axes(Qt::Horizontal).first());
                auto *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());

                if (axisX) {
                    axisX->setMin(now.addSecs(-60));
                    axisX->setMax(now.addSecs(0));
                }
                if (axisY) {
                    switch (i) {
                    case 0: axisY->setRange(0, 60); break;   // temperatura
                    case 1: axisY->setRange(0, 100); break;  // wilgotność
                    case 2: axisY->setRange(900, 1100); break; // ciśnienie
                    case 3: axisY->setRange(20, 100); break; // hałas
                    case 4: axisY->setRange(0, 5); break;  // AQI
                    case 5: axisY->setRange(0, 2000); break; // TVOC
                    case 6: axisY->setRange(400, 2000); break; // eCO₂
                    }
                }
            }
        }
    }
}

