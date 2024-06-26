#include <QWidget>
#include <QPainter>

class ArrowWidget : public QWidget {
    Q_OBJECT

public:
    ArrowWidget(QWidget *parent = nullptr) : QWidget(parent) {}

protected:
    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        drawArrow(painter, 0, Qt::red); // Body arrow
        drawArrow(painter, 45, Qt::green); // Camera arrow, 45 degrees
        drawArrow(painter, -45, Qt::blue); // Wrist arrow, -45 degrees
    }

private:
    void drawArrow(QPainter &painter, int angle, const QColor &color) {
        painter.save();
        painter.setRenderHint(QPainter::Antialiasing);
        painter.translate(width() / 2, height() / 2);
        painter.rotate(angle);

        QPolygon arrow;
        arrow << QPoint(-10, 0) << QPoint(0, -30) << QPoint(10, 0) << QPoint(0, 10) << QPoint(-10, 0);
        painter.setBrush(color);
        painter.drawPolygon(arrow);

        painter.restore();
    }
};
