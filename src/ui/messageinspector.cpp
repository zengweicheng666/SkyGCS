#include "messageinspector.h"

#include <QCheckBox>
#include <QHeaderView>
#include <QPlainTextEdit>
#include <QSplitter>
#include <QTableWidget>
#include <QTime>
#include <QVBoxLayout>

namespace skygcs {

MessageInspector::MessageInspector(MavlinkEndpoint* endpoint, QWidget* parent)
    : QWidget(parent)
    , endpoint_(endpoint)
{
    table_ = new QTableWidget(this);
    table_->setColumnCount(6);
    table_->setHorizontalHeaderLabels({tr("#"), tr("时间"), tr("MSG ID"), tr("名称"),
                                       tr("长度"), tr("方向")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);

    detail_ = new QPlainTextEdit(this);
    detail_->setReadOnly(true);
    detail_->setStyleSheet("QPlainTextEdit{background:#14181f;color:#d8dee9;"
                           "font-family:Consolas,monospace;font-size:12px;}");

    auto* splitter = new QSplitter(Qt::Vertical, this);
    splitter->addWidget(table_);
    splitter->addWidget(detail_);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(splitter);

    connect(endpoint_, &MavlinkEndpoint::messageReceived, this, &MessageInspector::onMessage);
    connect(table_, &QTableWidget::currentCellChanged, this,
            [this](int row, int, int, int) { onCurrentRowChanged(row); });
}

void MessageInspector::onMessage(const MavMessage& msg)
{
    if (count_ >= 2000)
        return;   // 防止无限增长 (演示足够)
    appendMessage(msg);
    if (table_->rowCount() > 0)
        table_->scrollToBottom();
}

void MessageInspector::appendMessage(const MavMessage& msg)
{
    const MsgDef* def = MavlinkCodec::findDef(msg.msgid);
    const QString name = def ? QString::fromLatin1(def->name) : QStringLiteral("?");
    const int row = table_->rowCount();
    table_->insertRow(row);
    table_->setItem(row, 0, new QTableWidgetItem(QString::number(count_)));
    table_->setItem(row, 1, new QTableWidgetItem(
        QTime::currentTime().toString(QStringLiteral("HH:mm:ss.zzz"))));
    table_->setItem(row, 2, new QTableWidgetItem(QString::number(msg.msgid)));
    table_->setItem(row, 3, new QTableWidgetItem(name));
    table_->setItem(row, 4, new QTableWidgetItem(QString::number(msg.size())));
    table_->setItem(row, 5, new QTableWidgetItem(
        QStringLiteral("S%1/C%2").arg(msg.sysid).arg(msg.compid)));
    ++count_;
}

void MessageInspector::onCurrentRowChanged(int row)
{
    if (row < 0 || row >= table_->rowCount())
        return;
    const QString name = table_->item(row, 3)->text();
    const QString sys = table_->item(row, 5)->text();
    detail_->setPlainText(QStringLiteral("选中消息: %1  (%2)\n\n")
                              .arg(name, sys));
}

} // namespace skygcs
