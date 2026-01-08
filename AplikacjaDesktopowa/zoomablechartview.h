#ifndef ZOOMABLECHARTVIEW_H
#define ZOOMABLECHARTVIEW_H

#include <QtCharts/QChartView>
#include <QtCharts/QDateTimeAxis>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDateTime>

class ZoomableChartView : public QChartView
{
public:
    ZoomableChartView(QChart *chart, QWidget *parent = nullptr)
        : QChartView(chart, parent),
        m_panning(false)
    {
        setRubberBand(QChartView::NoRubberBand);
        setDragMode(QGraphicsView::NoDrag);
        setMouseTracking(true);
    }

protected:

    // Rozróżnienie wykresów po tytule
    bool isHistoryChart() const
    {
        return chart() && chart()->title().contains("historia");
    }

    bool isLiveChart() const
    {
        return chart() && chart()->title().contains("sesja");
    }

    // Zoom kółkiem
    void wheelEvent(QWheelEvent *event) override
    {
        auto *axisX = qobject_cast<QDateTimeAxis*>(chart()->axes(Qt::Horizontal).first());
        if (!axisX) return;

        qint64 minT = axisX->min().toMSecsSinceEpoch();
        qint64 maxT = axisX->max().toMSecsSinceEpoch();
        qint64 range = maxT - minT;
        if (range <= 0) return;

        double factor = (event->angleDelta().y() > 0) ? 0.8 : 1.25;
        qint64 newRange = qMax<qint64>(1000, range * factor);

        qint64 center = (minT + maxT) / 2;
        qint64 newMin = center - newRange / 2;
        qint64 newMax = center + newRange / 2;

        axisX->setMin(QDateTime::fromMSecsSinceEpoch(newMin));
        axisX->setMax(QDateTime::fromMSecsSinceEpoch(newMax));

        // Formatowanie osi X
        if (isHistoryChart()) {
            // HISTORIA → zawsze data + czas
            if (newRange > 2LL * 86400LL * 1000LL)
                axisX->setFormat("dd.MM.yyyy");
            else if (newRange > 3600LL * 1000LL)
                axisX->setFormat("dd.MM hh:mm");
            else
                axisX->setFormat("dd.MM hh:mm:ss");
        }
        else if (isLiveChart()) {
            // SESJA → tylko godzina
            axisX->setFormat("hh:mm:ss");
        }

        event->accept();
    }

    // Panning - kliknięcie
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            m_panning = true;
            m_lastPos = event->pos();
            setCursor(Qt::ClosedHandCursor);
            event->accept();
            return;
        }
        QChartView::mousePressEvent(event);
    }

    // Panning - ruch
    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (m_panning) {
            auto *axisX = qobject_cast<QDateTimeAxis*>(chart()->axes(Qt::Horizontal).first());
            if (!axisX) return;

            QPoint delta = event->pos() - m_lastPos;
            m_lastPos = event->pos();

            QRectF plotArea = chart()->plotArea();
            if (plotArea.width() <= 0) return;

            qint64 minT = axisX->min().toMSecsSinceEpoch();
            qint64 maxT = axisX->max().toMSecsSinceEpoch();
            qint64 range = maxT - minT;
            if (range <= 0) return;

            double pxPerMs = plotArea.width() / double(range);
            qint64 dt = qint64(-delta.x() / pxPerMs);

            axisX->setMin(QDateTime::fromMSecsSinceEpoch(minT + dt));
            axisX->setMax(QDateTime::fromMSecsSinceEpoch(maxT + dt));

            event->accept();
            return;
        }

        QChartView::mouseMoveEvent(event);
    }

    // Panning - puszczenie
    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            m_panning = false;
            unsetCursor();
            event->accept();
            return;
        }
        QChartView::mouseReleaseEvent(event);
    }

private:
    bool m_panning;
    QPoint m_lastPos;
};

#endif // ZOOMABLECHARTVIEW_H
