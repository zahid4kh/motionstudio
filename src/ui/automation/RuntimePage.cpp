#include "RuntimePage.h"
#include "SignalTracePlot.h"
#include "physics/SimulationEngine.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace {
enum WCol { W_Plot, W_Name, W_Value, W_Count };
}

RuntimePage::RuntimePage(SceneGraph* scene, SimulationEngine* sim, QWidget* parent)
    : Page(scene, sim, parent)
{
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    auto* split = new QSplitter(Qt::Horizontal);
    root->addWidget(split);

    // ── Inputs ──────────────────────────────────────────────────────────────
    auto* inBox = new QGroupBox("Inputs");
    inBox->setMinimumWidth(230);
    auto* iv = new QVBoxLayout(inBox);
    iv->setContentsMargins(6, 12, 6, 6);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    m_inputHost = new QWidget;
    m_inputLayout = new QVBoxLayout(m_inputHost);
    m_inputLayout->setContentsMargins(0, 0, 0, 0);
    m_inputLayout->setSpacing(4);
    scroll->setWidget(m_inputHost);
    iv->addWidget(scroll);
    split->addWidget(inBox);

    // ── Watch ───────────────────────────────────────────────────────────────
    auto* watchBox = new QGroupBox("Watch");
    auto* wv = new QVBoxLayout(watchBox);
    wv->setContentsMargins(6, 12, 6, 6);
    m_filter = new QLineEdit;
    m_filter->setPlaceholderText("Filter…");
    m_filter->setClearButtonEnabled(true);
    wv->addWidget(m_filter);
    m_watch = new QTableWidget(0, W_Count);
    m_watch->setHorizontalHeaderLabels({"Plot", "Name", "Value"});
    m_watch->verticalHeader()->hide();
    m_watch->setAlternatingRowColors(true);
    m_watch->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_watch->setSelectionMode(QAbstractItemView::NoSelection);
    m_watch->verticalHeader()->setDefaultSectionSize(20);
    m_watch->setColumnWidth(W_Plot, 36);
    m_watch->setColumnWidth(W_Name, 150);
    m_watch->horizontalHeader()->setSectionResizeMode(W_Value, QHeaderView::Stretch);
    wv->addWidget(m_watch);
    split->addWidget(watchBox);

    // ── Status + trace ──────────────────────────────────────────────────────
    auto* right = new QSplitter(Qt::Vertical);
    auto* stBox = new QGroupBox("Sequences");
    auto* sv = new QVBoxLayout(stBox);
    sv->setContentsMargins(6, 12, 6, 6);
    m_header = new QLabel;
    m_header->setStyleSheet("color:#334;");
    sv->addWidget(m_header);
    m_status = new QTreeWidget;
    m_status->setColumnCount(2);
    m_status->setHeaderLabels({"Operation", "State"});
    m_status->header()->setSectionResizeMode(0, QHeaderView::Interactive);
    m_status->setColumnWidth(0, 200);
    m_status->setRootIsDecorated(true);
    sv->addWidget(m_status, 1);
    m_warnings = new QLabel;
    m_warnings->setWordWrap(true);
    m_warnings->setStyleSheet("color:#a05010; font-size:10px;");
    sv->addWidget(m_warnings);
    right->addWidget(stBox);

    auto* trBox = new QGroupBox("Trace");
    auto* tv = new QVBoxLayout(trBox);
    tv->setContentsMargins(6, 12, 6, 6);
    auto* tb = new QHBoxLayout;
    m_window = new QComboBox;
    m_window->addItems({"5 s", "10 s", "20 s", "60 s"});
    m_window->setCurrentIndex(2);
    auto* clr = new QPushButton("Clear");
    tb->addWidget(new QLabel("Window"));
    tb->addWidget(m_window);
    tb->addStretch();
    tb->addWidget(clr);
    tv->addLayout(tb);
    m_plot = new SignalTracePlot;
    tv->addWidget(m_plot, 1);
    right->addWidget(trBox);
    split->addWidget(right);

    split->setStretchFactor(0, 2);
    split->setStretchFactor(1, 3);
    split->setStretchFactor(2, 5);

    connect(m_filter, &QLineEdit::textChanged, this, [this]{ rebuildWatch(); refreshRuntime(); });
    connect(m_watch, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* it){
        if (m_building || it->column() != W_Plot) return;
        auto* name = m_watch->item(it->row(), W_Name);
        if (!name) return;
        if (it->checkState() == Qt::Checked) m_plotted.insert(name->text()); else m_plotted.remove(name->text());
        updateTraces();
    });
    connect(m_window, &QComboBox::currentIndexChanged, this, [this](int i){
        static const double w[] = {5, 10, 20, 60};
        m_plot->setWindow(w[i]);
    });
    connect(clr, &QPushButton::clicked, m_plot, &SignalTracePlot::clear);
    connect(m_sim->automation(), &AutomationEngine::scanned, this, &RuntimePage::onScanned);
    connect(m_sim, &SimulationEngine::activeChanged, this, [this](bool active){
        if (active) { m_plot->clear(); m_lastScan = -1; }
        refresh();
    });

    refresh();
}

void RuntimePage::refresh() {
    rebuildInputs();
    rebuildWatch();
    rebuildStatusTree();
    updateTraces();
    refreshRuntime();
}

void RuntimePage::setLocked(bool locked) {
    Page::setLocked(locked);
    refresh();
}

// ── Inputs ───────────────────────────────────────────────────────────────────

void RuntimePage::rebuildInputs() {
    m_inputs.clear();
    while (QLayoutItem* it = m_inputLayout->takeAt(0)) {
        if (it->widget()) it->widget()->deleteLater();
        delete it;
    }
    const bool live = m_sim->automation()->isActive();
    for (auto& [id, s] : m_scene->allSignals()) {
        if (s.kind != SignalKind::Input) continue;
        InputRow r;
        r.name = s.name;
        r.type = s.type;
        auto* row = new QWidget;
        auto* h = new QHBoxLayout(row);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(4);
        auto* lbl = new QLabel(s.name);
        lbl->setToolTip(s.description);
        QFont f = lbl->font(); f.setBold(true); lbl->setFont(f);
        lbl->setMinimumWidth(70);
        h->addWidget(lbl, 1);
        const QString name = s.name;
        if (s.type == SignalType::Bool) {
            r.toggle = new QPushButton("OFF");
            r.toggle->setCheckable(true);
            r.toggle->setToolTip("Latching switch");
            r.toggle->setFixedWidth(52);
            r.pulse = new QPushButton("Push");
            r.pulse->setToolTip("Momentary button: true while held (at least one scan)");
            r.pulse->setFixedWidth(52);
            connect(r.toggle, &QPushButton::clicked, this, [this, name](bool on){
                m_sim->automation()->setInput(name, on ? 1 : 0);
            });
            connect(r.pulse, &QPushButton::pressed, this, [this, name]{ m_sim->automation()->setInput(name, 1); });
            connect(r.pulse, &QPushButton::released, this, [this, name]{ m_sim->automation()->setInput(name, 0); });
            h->addWidget(r.pulse);
            h->addWidget(r.toggle);
        } else {
            r.spin = new QDoubleSpinBox;
            r.spin->setRange(-1e9, 1e9);
            r.spin->setDecimals(s.type == SignalType::Int ? 0 : 3);
            r.spin->setValue(s.initialValue);
            r.spin->setFixedWidth(100);
            r.spin->setKeyboardTracking(false);
            connect(r.spin, &QDoubleSpinBox::valueChanged, this, [this, name](double v){
                m_sim->automation()->setInput(name, v);
            });
            h->addWidget(r.spin);
        }
        for (QWidget* w : {static_cast<QWidget*>(r.toggle), static_cast<QWidget*>(r.pulse), static_cast<QWidget*>(r.spin)})
            if (w) w->setEnabled(live);
        m_inputLayout->addWidget(row);
        m_inputs.push_back(r);
    }
    if (m_inputs.empty()) {
        auto* l = new QLabel("No Input signals. Add them on the Signals tab (e.g. Start, Stop, EStop).");
        l->setWordWrap(true);
        l->setStyleSheet("color:#889;");
        m_inputLayout->addWidget(l);
    } else if (!live) {
        auto* l = new QLabel("Press Play or Step to operate the inputs.");
        l->setWordWrap(true);
        l->setStyleSheet("color:#889; font-size:10px;");
        m_inputLayout->addWidget(l);
    }
    m_inputLayout->addStretch();
}

// ── Watch ────────────────────────────────────────────────────────────────────

void RuntimePage::rebuildWatch() {
    AutomationEngine* ae = m_sim->automation();
    std::vector<AutomationEngine::Symbol> syms = ae->isActive() ? ae->symbols()
                                                                : AutomationEngine::buildSymbols(*m_scene);
    const QString filter = m_filter->text().trimmed();
    m_building = true;
    m_watch->setRowCount(0);
    int row = 0;
    for (const auto& s : syms) {
        if (!filter.isEmpty() && !s.name.contains(filter, Qt::CaseInsensitive)) continue;
        m_watch->insertRow(row);
        auto* plot = new QTableWidgetItem;
        plot->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        plot->setCheckState(m_plotted.contains(s.name) ? Qt::Checked : Qt::Unchecked);
        m_watch->setItem(row, W_Plot, plot);
        auto* name = new QTableWidgetItem(s.name);
        name->setToolTip(s.help);
        name->setData(Qt::UserRole, s.isBool);
        name->setData(Qt::UserRole + 1, int(s.type));
        if (s.kind == AutomationEngine::SymbolKind::Signal) {
            QFont f = name->font(); f.setBold(true); name->setFont(f);
        } else {
            name->setForeground(QColor("#556"));
        }
        m_watch->setItem(row, W_Name, name);
        auto* val = new QTableWidgetItem("—");
        val->setFont(QFont("monospace"));
        m_watch->setItem(row, W_Value, val);
        ++row;
    }
    m_building = false;
}

void RuntimePage::updateTraces() {
    // Keep the order in which signals appear in the symbol table
    AutomationEngine* ae = m_sim->automation();
    std::vector<AutomationEngine::Symbol> syms = ae->isActive() ? ae->symbols()
                                                                : AutomationEngine::buildSymbols(*m_scene);
    QStringList names;
    std::vector<bool> isBool;
    m_traceSlots.clear();
    QSet<QString> still;
    for (int i = 0; i < int(syms.size()); ++i) {
        if (!m_plotted.contains(syms[size_t(i)].name)) continue;
        names << syms[size_t(i)].name;
        isBool.push_back(syms[size_t(i)].isBool);
        m_traceSlots.push_back(ae->isActive() ? i : -1);
        still.insert(syms[size_t(i)].name);
    }
    m_plotted = still;
    m_plot->setTraces(names, isBool);
}

void RuntimePage::onScanned() {
    AutomationEngine* ae = m_sim->automation();
    if (m_traceSlots.empty()) return;
    if (!m_traceSlots.empty() && m_traceSlots.front() < 0) updateTraces();
    std::vector<double> vals;
    vals.reserve(m_traceSlots.size());
    for (int slot : m_traceSlots) vals.push_back(ae->value(slot));
    m_plot->addSample(ae->time(), vals);
}

// ── Status ───────────────────────────────────────────────────────────────────

void RuntimePage::rebuildStatusTree() {
    m_status->clear();
    for (auto& [id, q] : m_scene->allSequences()) {
        auto* top = new QTreeWidgetItem(m_status, {q.name, ""});
        top->setData(0, Qt::UserRole, id);
        QFont f = top->font(0); f.setBold(true); top->setFont(0, f);
        for (int i = 0; i < int(q.operations.size()); ++i) {
            const Operation& op = q.operations[size_t(i)];
            auto* it = new QTreeWidgetItem(top, {QString("%1  %2").arg(i + 1)
                .arg(op.name.isEmpty() ? operationTypeName(op.type) : op.name), ""});
            it->setData(0, Qt::UserRole, op.id);
        }
        top->setExpanded(true);
    }
}

void RuntimePage::refreshRuntime() {
    AutomationEngine* ae = m_sim->automation();
    const bool live = ae->isActive();

    // Header
    if (live) {
        QStringList act = ae->activeOperationLabels();
        m_header->setText(QString("<b>t = %1 s</b> &nbsp; scans: %2 &nbsp; %3")
            .arg(ae->time(), 0, 'f', 2).arg(ae->scanCount())
            .arg(m_sim->isRunning() ? "<span style='color:#1a7a30'>● running</span>"
                                    : "<span style='color:#a06010'>❚❚ paused</span>"));
        const auto& w = ae->warnings();
        m_warnings->setText(w.isEmpty() ? QString() : "⚠ " + w.mid(std::max(0, int(w.size()) - 3)).join("\n⚠ "));
    } else {
        m_header->setText("<span style='color:#889'>Simulation stopped — press Play or Step.</span>");
        m_warnings->clear();
    }

    // Input widgets reflect the live value (e.g. after actions write to them)
    for (auto& r : m_inputs) {
        double v = ae->displayValue(r.name);
        if (r.toggle && !r.pulse->isDown()) {
            QSignalBlocker b(r.toggle);
            r.toggle->setChecked(live && v != 0);
            r.toggle->setText(live && v != 0 ? "ON" : "OFF");
        }
    }

    // Watch values
    for (int r = 0; r < m_watch->rowCount(); ++r) {
        auto* name = m_watch->item(r, W_Name);
        auto* val  = m_watch->item(r, W_Value);
        if (!live) { val->setText("—"); val->setBackground(QBrush()); continue; }
        bool isBool = name->data(Qt::UserRole).toBool();
        double v = ae->value(name->text());
        val->setText(autoui::formatValue(v, isBool, SignalType(name->data(Qt::UserRole + 1).toInt())));
        if (isBool) val->setBackground(v != 0 ? QColor("#c8f0c8") : QBrush());
    }

    // Status tree
    for (int i = 0; i < m_status->topLevelItemCount(); ++i) {
        auto* top = m_status->topLevelItem(i);
        int sid = top->data(0, Qt::UserRole).toInt();
        if (!live) {
            top->setText(1, "");
            for (int k = 0; k < top->childCount(); ++k) {
                top->child(k)->setText(1, "");
                top->child(k)->setBackground(1, QBrush());
            }
            continue;
        }
        auto ss = ae->seqStatus(sid);
        static const char* names[] = {"idle", "running", "finished"};
        top->setText(1, QString("%1 · cycles %2 · %3").arg(names[int(ss.state)]).arg(ss.cycles).arg(ss.detail));
        for (int k = 0; k < top->childCount(); ++k) {
            auto* it = top->child(k);
            auto st = ae->opStatus(sid, it->data(0, Qt::UserRole).toInt());
            QString txt;
            QColor bg;
            switch (st.state) {
                case AutomationEngine::OpState::Waiting:
                    txt = "waiting" + (st.detail.isEmpty() ? QString() : " — " + st.detail);
                    bg = QColor("#f4f4f4");
                    break;
                case AutomationEngine::OpState::Running:
                    txt = QString("running %1%").arg(int(st.progress * 100)) +
                          (st.detail.isEmpty() ? QString() : " — " + st.detail);
                    bg = QColor("#cfe3fa");
                    break;
                case AutomationEngine::OpState::Done:
                    txt = QString("done in %1 s").arg(st.endTime - st.startTime, 0, 'f', 2);
                    bg = QColor("#d4f0d4");
                    break;
            }
            it->setText(1, txt);
            it->setBackground(1, bg);
        }
    }
    m_plot->update();
}
