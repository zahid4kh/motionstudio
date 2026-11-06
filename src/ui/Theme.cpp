#include "Theme.h"
#include <QAbstractButton>
#include <QApplication>
#include <QDir>
#include <QFont>
#include <QFontDatabase>
#include <QHash>
#include <QIconEngine>
#include <QLabel>
#include <QPainter>
#include <QPalette>
#include <QPixmap>
#include <QSettings>
#include <QStyle>
#include <QStyleFactory>
#include <QWidget>
#include <algorithm>

namespace {

// Monochrome dark (QODE's "mono-dark"), with saturated semantic colours that
// read on the dark surfaces.
const Theme::Colors kDark{
    true,
    QColor(0x0c, 0x0c, 0x0c), // frame
    QColor(0x12, 0x12, 0x12), // island
    QColor(0x1b, 0x1b, 0x1b), // panel
    QColor(0x22, 0x22, 0x22), // hover
    QColor(0x2a, 0x2a, 0x2a), // hoverStrong
    QColor(0x3a, 0x3a, 0x3a), // selection
    QColor(0x43, 0x43, 0x43), // selectionHover
    QColor(0x26, 0x26, 0x26), // border
    QColor(0x30, 0x30, 0x30), // inputBorder
    QColor(0x55, 0x55, 0x55), // handle
    QColor(0xd4, 0xd4, 0xd4), // text
    QColor(0x8a, 0x8a, 0x8a), // muted
    QColor(0xe6, 0xe6, 0xe6), // accent
    QColor(0xff, 0xff, 0xff), // accentHover
    QColor(0x12, 0x12, 0x12), // onAccent
    QColor(110, 220, 130),    // success
    QColor(105, 170, 255),    // info
    QColor(255, 180, 100),    // warning
    QColor(255, 105, 105),    // danger
    QColor(240, 150, 60),     // physics
    QColor(230, 64, 64),      // axisX
    QColor(77, 204, 77),      // axisY
    QColor(77, 125, 242),     // axisZ
    QColor(0x17, 0x17, 0x17), // viewport
    QColor(0x2e, 0x2e, 0x2e), // grid
    QColor(44, 60, 92),       // gridFloor
    QColor(90, 130, 210),     // floorFill
    {QColor("#5aa0ff"), QColor("#ff8a5c"), QColor("#5fd47e"), QColor("#c08cff"),
     QColor("#f2c94c"), QColor("#4fd1d9"), QColor("#ff6fa8"), QColor("#a0aec0")},
};

// Monochrome light (QODE's "mono-light": mid greys, not white), with darker
// semantic colours.
const Theme::Colors kLight{
    false,
    QColor(0xa9, 0xa9, 0xa9), // frame
    QColor(0xcf, 0xcf, 0xcf), // island
    QColor(0xc8, 0xc8, 0xc8), // panel
    QColor(0xbf, 0xbf, 0xbf), // hover
    QColor(0xb5, 0xb5, 0xb5), // hoverStrong
    QColor(0xa4, 0xa4, 0xa4), // selection
    QColor(0x9a, 0x9a, 0x9a), // selectionHover
    QColor(0xa8, 0xa8, 0xa8), // border
    QColor(0x9e, 0x9e, 0x9e), // inputBorder
    QColor(0x7c, 0x7c, 0x7c), // handle
    QColor(0x1a, 0x1a, 0x1a), // text
    QColor(0x55, 0x55, 0x55), // muted
    QColor(0x24, 0x24, 0x24), // accent
    QColor(0x3a, 0x3a, 0x3a), // accentHover
    QColor(0xff, 0xff, 0xff), // onAccent
    QColor(25, 125, 60),      // success
    QColor(25, 90, 200),      // info
    QColor(165, 85, 0),       // warning
    QColor(200, 35, 35),      // danger
    QColor(190, 95, 0),       // physics
    QColor(199, 41, 41),      // axisX
    QColor(31, 135, 41),      // axisY
    QColor(41, 79, 214),      // axisZ
    QColor(0xd6, 0xd6, 0xd6), // viewport
    QColor(0xb4, 0xb4, 0xb4), // grid
    QColor(112, 132, 178),    // gridFloor
    QColor(60, 100, 190),     // floorFill
    {QColor("#1a6fda"), QColor("#d0582a"), QColor("#2a9a48"), QColor("#9a3ad0"),
     QColor("#a87a00"), QColor("#108a9a"), QColor("#c03070"), QColor("#4a5a7a")},
};

Theme::Mode s_mode = Theme::Mode::Dark;

QSettings preferences() {
    // App-wide preferences only (the theme); project state lives in .mstudio files
    return QSettings("MotionStudio", "MotionStudio");
}

QString styleSheet() {
    QString css = QStringLiteral(R"(
QMainWindow, QWidget#frame { background: @frame@; }
QMainWindow::separator { background: transparent; width: 8px; height: 8px; }
QDialog, QMessageBox, QInputDialog, QProgressDialog { background: @panel@; }
QWidget { color: @fg@; }
QWidget#island, QWidget#islandHeader { background: @island@; }
QLabel#islandTitle { color: @muted@; font-size: 11px; font-weight: 700; }
QToolButton#islandButton { padding: 3px; border-radius: 6px; }

QMenuBar { background: @frame@; color: @fg@; padding: 2px 0; }
QMenuBar::item { padding: 4px 10px; background: transparent; border-radius: 6px; }
QMenuBar::item:selected { background: @hover@; }
QMenu { background: @panel@; color: @fg@; border: 1px solid @handle@; border-radius: 10px; padding: 5px; }
QMenu::item { padding: 6px 26px 6px 10px; margin: 1px 0; border-radius: 6px; background: transparent; }
QMenu::item:selected { background: @sel@; }
QMenu::item:disabled { color: @muted@; }
QMenu::icon { padding-left: 8px; }
QMenu::separator { height: 1px; background: @border@; margin: 5px 8px; }
QMenu::indicator { width: 14px; height: 14px; margin-left: 6px; }
QMenu::right-arrow { image: url(@ui@/chevron-right.svg); width: 12px; height: 12px; margin-right: 6px; }
QStatusBar { background: @frame@; color: @muted@; padding-left: 6px; }
QStatusBar[tone="success"] { color: @success@; }
QStatusBar[tone="error"] { color: @danger@; font-weight: 600; }
QStatusBar::item { border: none; }
QStatusBar QLabel { color: @muted@; padding: 0 8px; }
QLabel#timeChip { background: @island@; color: @fg@; border: 1px solid @border@; border-radius: 9px;
    padding: 1px 10px; margin: 2px 0; font-family: monospace; }
QLabel#recChip { color: @muted@; font-weight: 700; padding: 0 2px; }
QLabel#recChip[active="true"] { color: @danger@; }
QToolTip { background: @panel@; color: @fg@; border: 1px solid @handle@; border-radius: 8px; padding: 6px 8px; }

QToolBar { background: transparent; border: none; spacing: 2px; padding: 0; }
QToolBar::separator { background: @border@; width: 1px; margin: 8px 6px; }
QToolButton { background: transparent; color: @fg@; border: 1px solid transparent; border-radius: 8px; padding: 4px 6px; }
QToolButton:hover { background: @hover@; }
QToolButton:pressed, QToolButton:checked { background: @sel@; }
QToolButton:checked:hover { background: @selhover@; }
QToolButton:disabled { color: @muted@; }
QToolBar QToolButton { padding: 4px 3px; font-size: 11px; min-width: 42px; }
QToolButton#primaryTool { background: @accent@; color: @onaccent@; font-weight: 700; }
QToolButton#primaryTool:hover { background: @accenthover@; }
QToolButton#primaryTool:disabled { background: @hover@; color: @muted@; }
QToolButton[popupMode="2"] { padding-right: 18px; }
QToolButton::menu-indicator { image: url(@ui@/chevron-down.svg); width: 10px; height: 10px;
    subcontrol-origin: padding; subcontrol-position: center right; right: 5px; }
QToolButton#problems { font-size: 11px; padding: 2px 8px; }
QToolButton#menuButton { background: @hover@; border: 1px solid @border@; padding: 4px 10px; }
QToolButton#menuButton[popupMode="2"] { padding-right: 20px; }
QToolButton#menuButton:hover { background: @hoverstrong@; border-color: @handle@; }
QToolButton#menuButton:disabled { background: transparent; }
QToolButton#problems[tone="ok"] { color: @success@; }
QToolButton#problems[tone="bad"] { color: @danger@; font-weight: 700; }

QLabel#hint { color: @muted@; font-size: 11px; }
QLabel#subtle { color: @muted@; }
QLabel#warningText { color: @warning@; font-size: 11px; }
QLabel#fieldLabel { color: @muted@; }
QLabel#nameTitle { font-size: 15px; font-weight: 700; }
QLabel#mono { font-family: monospace; font-weight: 600; }
QLabel#monoHint { font-family: monospace; color: @muted@; font-size: 11px; }
QLabel#axisX { color: @axisx@; font-weight: 700; font-size: 11px; }
QLabel#axisY { color: @axisy@; font-weight: 700; font-size: 11px; }
QLabel#axisZ { color: @axisz@; font-weight: 700; font-size: 11px; }
QLabel#subheading { color: @muted@; font-size: 11px; font-weight: 700; padding: 4px 0 0 0; }
QLabel#dialogTitle { font-size: 15px; font-weight: 700; }
QLabel#importStep { color: @muted@; font-size: 11px; font-weight: 600; }
QLabel#importStep[done="true"] { color: @fg@; }
QLabel#importStep[active="true"] { color: @accent@; font-weight: 700; }
QLabel#importPercent { font-size: 20px; font-weight: 700; }
QLabel#templateDesc { background: @island@; color: @fg@; border: 1px solid @border@; border-radius: 10px;
    padding: 10px 12px; min-height: 60px; }
QTextBrowser#helpView { background: @island@; color: @fg@; border: 1px solid @border@; border-radius: 10px; }
QWidget#card { background: @panel@; border: 1px solid @border@; border-radius: 12px; }
QWidget#card QLabel { background: transparent; }

QGroupBox { background: @panel@; border: 1px solid @border@; border-radius: 12px; margin-top: 24px;
    padding: 8px 4px 4px 4px; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 6px; top: 4px;
    color: @muted@; font-weight: 700; }

QLineEdit, QAbstractSpinBox, QComboBox {
    background: @island@; color: @fg@; border: 1px solid @inputborder@; border-radius: 8px; padding: 3px 8px;
    min-height: 18px; selection-background-color: @sel@; selection-color: @fg@; }
QLineEdit:hover, QAbstractSpinBox:hover, QComboBox:hover { border-color: @handle@; }
QLineEdit:focus, QAbstractSpinBox:focus, QComboBox:focus { border-color: @accent@; }
QLineEdit:disabled, QAbstractSpinBox:disabled, QComboBox:disabled { color: @muted@; border-color: @border@; }
ExpressionEdit { font-family: monospace; }
ExpressionEdit[error="true"] { border-color: @danger@; background: @dangersoft@; }
QAbstractSpinBox { padding: 3px 4px; padding-right: 18px; }
QAbstractSpinBox[buttonSymbols="2"] { padding-right: 4px; }
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button { subcontrol-origin: border; width: 16px;
    border: none; background: transparent; }
QAbstractSpinBox::up-button { subcontrol-position: top right; margin: 2px 2px 0 0; }
QAbstractSpinBox::down-button { subcontrol-position: bottom right; margin: 0 2px 2px 0; }
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover { background: @sel@; border-radius: 4px; }
QAbstractSpinBox::up-arrow { image: url(@ui@/chevron-up.svg); width: 10px; height: 10px; }
QAbstractSpinBox::down-arrow { image: url(@ui@/chevron-down.svg); width: 10px; height: 10px; }
QAbstractSpinBox::up-arrow:disabled, QAbstractSpinBox::down-arrow:disabled { image: none; }
QComboBox { padding-right: 24px; }
QComboBox::drop-down { border: none; width: 24px; }
QComboBox::down-arrow { image: url(@ui@/chevron-down.svg); width: 12px; height: 12px; }
QComboBox { combobox-popup: 0; }
QComboBoxPrivateContainer { background: @panel@; border: 1px solid @handle@; border-radius: 0; padding: 0; }
QComboBox QAbstractItemView { background: @panel@; color: @fg@; border: none; border-radius: 0;
    padding: 4px; outline: 0; selection-background-color: @sel@; selection-color: @fg@; }
QComboBox QAbstractItemView::item { min-height: 24px; padding: 0 10px; border-radius: 6px; color: @fg@; }
QComboBox QAbstractItemView::item:hover { background: @hover@; }
QComboBox QAbstractItemView::item:selected { background: @sel@; color: @fg@; }
QTableView QLineEdit, QTableView QComboBox, QTableView QAbstractSpinBox {
    border: none; border-radius: 0; background: transparent; padding: 0 4px; min-height: 0; }
QTableView QComboBox { padding: 0 2px; padding-right: 16px; }
QTableView QComboBox::drop-down { width: 16px; }
QTableView ExpressionEdit[error="true"] { background: @dangersoft@; }

QPushButton { background: @hover@; color: @fg@; border: 1px solid @border@; border-radius: 8px; padding: 5px 12px; }
QPushButton:hover { border-color: @handle@; background: @hoverstrong@; }
QPushButton:pressed { background: @sel@; }
QPushButton:checked { background: @accent@; color: @onaccent@; border-color: @accent@; font-weight: 700; }
QPushButton:disabled { color: @muted@; background: transparent; }
QLabel[error="true"] { color: @danger@; }
QPushButton[danger="true"] { color: @danger@; }
QPushButton[danger="true"]:hover { background: @dangersoft@; border-color: @danger@; }
QPushButton[danger="true"]:disabled { color: @muted@; }
QDialogButtonBox { dialogbuttonbox-buttons-have-icons: 0; }
QPushButton#primaryButton, QDialogButtonBox QPushButton:default {
    background: @accent@; color: @onaccent@; border: 1px solid @accent@; font-weight: 600; }
QPushButton#primaryButton:hover, QDialogButtonBox QPushButton:default:hover {
    background: @accenthover@; border-color: @accenthover@; }
QPushButton#primaryButton:disabled, QDialogButtonBox QPushButton:default:disabled {
    background: @hover@; color: @muted@; border-color: @border@; }
QPushButton#iconButton { padding: 4px; min-width: 22px; }
QPushButton#segment { background: @hover@; border: 1px solid @border@; border-radius: 8px; padding: 6px 10px; font-weight: 600; }
QPushButton#segment:hover { background: @hoverstrong@; border-color: @handle@; }
QPushButton#segment:checked { background: @accent@; color: @onaccent@; border-color: @accent@; }
QLabel#statValue { font-size: 14px; font-weight: 700; }
QLabel#dimSymbol { font-family: monospace; font-weight: 700; color: @info@; }
QPushButton#viewButton { background: @overlay@; color: @fg@; border: 1px solid @border@; border-radius: 8px;
    padding: 2px 8px; font-size: 11px; font-weight: 600; }
QPushButton#viewButton:hover { background: @hoverstrong@; border-color: @handle@; }
QPushButton#viewButton:pressed { background: @sel@; }
QPushButton#viewButton[active="true"] { background: @accent@; color: @onaccent@; border-color: @accent@; }
QFrame#lightingPanel { background: @panel@; border: 1px solid @handle@; }
QFrame#assemblePanel { background: @panel@; border: 1px solid @handle@; border-radius: 12px; }
QLabel#viewOverlay { background: @overlay@; color: @fg@; border: 1px solid @border@; border-radius: 10px;
    padding: 6px 10px; font-size: 11px; }

QCheckBox, QRadioButton { spacing: 8px; background: transparent; }
QCheckBox::indicator, QRadioButton::indicator { width: 14px; height: 14px; border: 1px solid @handle@;
    background: @island@; }
QCheckBox::indicator { border-radius: 4px; }
QRadioButton::indicator { border-radius: 8px; }
QCheckBox::indicator:hover, QRadioButton::indicator:hover { border-color: @accent@; }
QCheckBox::indicator:checked { background: @accent@; border-color: @accent@; image: url(@ui@/check.svg); }
QRadioButton::indicator:checked { background: @island@; border: 4px solid @accent@; width: 8px; height: 8px; }
QCheckBox:disabled, QRadioButton:disabled { color: @muted@; }
QCheckBox::indicator:disabled { border-color: @border@; }
QCheckBox::indicator:checked:disabled { background: @handle@; border-color: @handle@; }

QSlider::groove:horizontal { height: 4px; background: @hoverstrong@; border-radius: 2px; }
QSlider::sub-page:horizontal { background: @handle@; border-radius: 2px; }
QSlider::handle:horizontal { background: @accent@; width: 12px; height: 12px; margin: -5px 0; border-radius: 6px; }
QSlider::handle:horizontal:hover { background: @accenthover@; }
QSlider::handle:horizontal:disabled { background: @handle@; }

QProgressBar { background: @island@; border: 1px solid @border@; border-radius: 6px; text-align: center; max-height: 10px; }
QProgressBar::chunk { background: @accent@; border-radius: 5px; }

QAbstractItemView { background: @island@; alternate-background-color: @altrow@; border: 1px solid @border@;
    border-radius: 8px; outline: 0; gridline-color: @border@; selection-background-color: @sel@;
    selection-color: @fg@; }
QAbstractItemView::item:hover { background: @hover@; }
QAbstractItemView::indicator { width: 14px; height: 14px; border: 1px solid @handle@; border-radius: 4px;
    background: @island@; }
QAbstractItemView::indicator:hover { border-color: @accent@; }
QAbstractItemView::indicator:checked { background: @accent@; border-color: @accent@; image: url(@ui@/check.svg); }
QAbstractItemView::item:selected { background: @sel@; color: @fg@; }
QListView::item { padding: 4px 6px; border-radius: 6px; }
QTreeView::item { padding: 3px 2px; }
QTreeView::branch { background: transparent; }
QTreeView::branch:has-children:!has-siblings:closed, QTreeView::branch:closed:has-children:has-siblings {
    image: url(@ui@/chevron-right.svg); }
QTreeView::branch:open:has-children:!has-siblings, QTreeView::branch:open:has-children:has-siblings {
    image: url(@ui@/chevron-down.svg); }
QTreeWidget#hierarchyTree { background: transparent; border: none; padding: 0 6px; }
QTreeWidget#hierarchyTree::item { padding: 5px 4px; border: none; }
QTreeWidget#hierarchyTree::item:selected { background: @sel@; }
QTreeWidget#hierarchyTree::item:first { border-top-left-radius: 8px; border-bottom-left-radius: 8px; }
QTreeWidget#hierarchyTree::item:last { border-top-right-radius: 8px; border-bottom-right-radius: 8px; }
QListWidget#templateList::item { padding: 8px 10px; margin: 2px; }
QHeaderView { background: transparent; border: none; border-radius: 0; }
QHeaderView::section { background: @panel@; color: @muted@; border: none; border-right: 1px solid @border@;
    border-bottom: 1px solid @border@; padding: 4px 5px; font-weight: 600; }
QHeaderView::section:first { border-top-left-radius: 8px; }
QTableCornerButton::section { background: @panel@; border: none; }

QTabWidget::pane { border: none; border-top: 1px solid @border@; }
QTabBar { background: transparent; }
QTabBar::tab { background: transparent; color: @muted@; border: none; padding: 5px 12px; margin: 6px 2px 6px 0;
    border-radius: 8px; font-weight: 600; }
QTabBar::tab:first { margin-left: 8px; }
QTabBar::tab:hover { background: @hover@; color: @fg@; }
QTabBar::tab:selected { background: @sel@; color: @fg@; }

QSplitter::handle { background: transparent; }
QSplitter::handle:hover { background: @border@; }
QScrollArea { background: transparent; border: none; }
QScrollArea > QWidget#qt_scrollarea_viewport, QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical { background: transparent; width: 12px; margin: 0; }
QScrollBar:horizontal { background: transparent; height: 12px; margin: 0; }
QScrollBar::handle { background: @handle@; border-radius: 4px; min-height: 24px; min-width: 24px; margin: 2px; }
QScrollBar::handle:hover { background: @muted@; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
)");

    const Theme::Colors& c = Theme::colors();
    auto hex  = [](const QColor& color) { return color.name(QColor::HexRgb); };
    auto rgba = [](const QColor& color, qreal a) {
        return QString("rgba(%1,%2,%3,%4)").arg(color.red()).arg(color.green()).arg(color.blue()).arg(int(a * 255));
    };
    QColor alt = c.island.lighter(c.dark ? 118 : 104);
    const QPair<QString, QString> map[] = {
        {"@ui@",           c.dark ? ":/icons/ui/dark" : ":/icons/ui/light"},
        {"@frame@",        hex(c.frame)},
        {"@island@",       hex(c.island)},
        {"@panel@",        hex(c.panel)},
        {"@hoverstrong@",  hex(c.hoverStrong)},
        {"@hover@",        hex(c.hover)},
        {"@selhover@",     hex(c.selectionHover)},
        {"@sel@",          hex(c.selection)},
        {"@border@",       hex(c.border)},
        {"@inputborder@",  hex(c.inputBorder)},
        {"@handle@",       hex(c.handle)},
        {"@fg@",           hex(c.text)},
        {"@muted@",        hex(c.muted)},
        {"@accenthover@",  hex(c.accentHover)},
        {"@accent@",       hex(c.accent)},
        {"@onaccent@",     hex(c.onAccent)},
        {"@success@",      hex(c.success)},
        {"@warning@",      hex(c.warning)},
        {"@dangersoft@",   rgba(c.danger, 0.14)},
        {"@danger@",       hex(c.danger)},
        {"@info@",         hex(c.info)},
        {"@axisx@",        hex(c.axisX)},
        {"@axisy@",        hex(c.axisY)},
        {"@axisz@",        hex(c.axisZ)},
        {"@altrow@",       hex(alt)},
        {"@overlay@",      rgba(c.panel, 0.88)},
    };
    for (const auto& pair : map)
        css.replace(pair.first, pair.second);
    return css;
}

QPalette palette() {
    const Theme::Colors& c = Theme::colors();
    QPalette p;
    p.setColor(QPalette::Window,          c.panel);
    p.setColor(QPalette::WindowText,      c.text);
    p.setColor(QPalette::Base,            c.island);
    p.setColor(QPalette::AlternateBase,   c.island.lighter(c.dark ? 118 : 104));
    p.setColor(QPalette::Text,            c.text);
    p.setColor(QPalette::Button,          c.panel);
    p.setColor(QPalette::ButtonText,      c.text);
    p.setColor(QPalette::BrightText,      c.dark ? Qt::white : Qt::black);
    p.setColor(QPalette::ToolTipBase,     c.panel);
    p.setColor(QPalette::ToolTipText,     c.text);
    p.setColor(QPalette::Highlight,       c.selection);
    p.setColor(QPalette::HighlightedText, c.text);
    p.setColor(QPalette::PlaceholderText, c.muted);
    p.setColor(QPalette::Link,            c.info);
    p.setColor(QPalette::Mid,             c.border);
    p.setColor(QPalette::Dark,            c.frame);
    for (QPalette::ColorRole role : {QPalette::Text, QPalette::WindowText, QPalette::ButtonText})
        p.setColor(QPalette::Disabled, role, c.muted);
    return p;
}

void applyLook() {
    QApplication::setPalette(palette());
    qApp->setStyleSheet(styleSheet());
}

QColor tintColor(Theme::Tint tint) {
    const Theme::Colors& c = Theme::colors();
    switch (tint) {
        case Theme::Tint::Muted:    return c.muted;
        case Theme::Tint::OnAccent: return c.onAccent;
        case Theme::Tint::Success:  return c.success;
        case Theme::Tint::Warning:  return c.warning;
        case Theme::Tint::Danger:   return c.danger;
        case Theme::Tint::Physics:  return c.physics;
        case Theme::Tint::Text:     break;
    }
    return c.text;
}

// A white Lucide SVG recoloured when it is drawn, in the colour of the current theme
class TintedIconEngine : public QIconEngine {
public:
    TintedIconEngine(const QString& name, Theme::Tint tint)
        : m_source(QString(":/icons/%1.svg").arg(name)), m_name(name), m_tint(tint) {}

    void paint(QPainter* painter, const QRect& rect, QIcon::Mode mode, QIcon::State state) override {
        const qreal dpr = painter->device() ? painter->device()->devicePixelRatioF() : 1.0;
        QPixmap pm = pixmap(rect.size() * dpr, mode, state);
        pm.setDevicePixelRatio(dpr);
        painter->drawPixmap(rect, pm);
    }

    QPixmap pixmap(const QSize& size, QIcon::Mode mode, QIcon::State) override {
        QColor color = tintColor(m_tint);
        if (mode == QIcon::Disabled) {
            color = Theme::colors().muted;
            color.setAlphaF(0.55f);
        }
        const QString key = QString("%1x%2|%3").arg(size.width()).arg(size.height()).arg(color.name(QColor::HexArgb));
        auto it = m_cache.constFind(key);
        if (it != m_cache.cend()) return *it;
        // Rasterised by Qt's SVG icon engine, then recoloured
        QPixmap pm = m_source.pixmap(size);
        if (pm.isNull()) return pm;
        QPainter painter(&pm);
        painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
        painter.fillRect(pm.rect(), color);
        painter.end();
        m_cache.insert(key, pm);
        return pm;
    }

    QSize actualSize(const QSize& size, QIcon::Mode, QIcon::State) override {
        const int side = std::min(size.width(), size.height());
        return QSize(side, side);
    }

    QIconEngine* clone() const override { return new TintedIconEngine(*this); }
    QString key() const override { return "motionstudio-tinted"; }
    QString iconName() override { return m_name; }

private:
    QIcon       m_source;
    QString     m_name;
    Theme::Tint m_tint;
    QHash<QString, QPixmap> m_cache;
};

} // namespace

const Theme::Colors& Theme::colors() {
    return s_mode == Mode::Dark ? kDark : kLight;
}

Theme::Mode Theme::mode() {
    return s_mode;
}

Theme::Notifier* Theme::notifier() {
    static Notifier instance;
    return &instance;
}

void Theme::apply(QApplication& app) {
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    const QStringList fonts = QDir(":/fonts").entryList({"*.ttf"});
    for (const QString& file : fonts)
        QFontDatabase::addApplicationFont(":/fonts/" + file);
    QFont ui("Plus Jakarta Sans");
    ui.setPointSizeF(9.5);
    app.setFont(ui);

    // Dark unless light was chosen
    s_mode = preferences().value("ui/theme").toString() == QLatin1String("light") ? Mode::Light : Mode::Dark;
    applyLook();
}

void Theme::setMode(Mode mode) {
    if (mode == s_mode) return;
    s_mode = mode;
    preferences().setValue("ui/theme", mode == Mode::Light ? "light" : "dark");
    applyLook();
    emit notifier()->changed();
    for (QWidget* w : QApplication::allWidgets())
        w->update();
}

void Theme::toggle() {
    setMode(s_mode == Mode::Dark ? Mode::Light : Mode::Dark);
}

QIcon Theme::icon(const QString& name, Tint tint) {
    return QIcon(new TintedIconEngine(name, tint));
}

void Theme::setLabelIcon(QLabel* label, const QString& name, int size, Tint tint) {
    auto render = [label] {
        const QIcon i = icon(label->property("themeIcon").toString(), Tint(label->property("themeTint").toInt()));
        const int sz = label->property("themeIconSize").toInt();
        label->setPixmap(i.pixmap(QSize(sz, sz), label->devicePixelRatioF()));
    };
    const bool connected = label->property("themeIcon").isValid();
    label->setProperty("themeIcon", name);
    label->setProperty("themeIconSize", size);
    label->setProperty("themeTint", int(tint));
    render();
    if (!connected) QObject::connect(notifier(), &Notifier::changed, label, render);
}

QString Theme::css(const QColor& color) {
    return color.name(QColor::HexRgb);
}

QColor Theme::soft(const QColor& color, qreal alpha) {
    QColor c = color;
    c.setAlphaF(float(alpha));
    return c;
}

void Theme::repolish(QWidget* w) {
    w->style()->unpolish(w);
    w->style()->polish(w);
    w->update();
}

void Theme::segmentIcon(QAbstractButton* button, const QString& name) {
    auto apply = [button, name](bool checked) { button->setIcon(Theme::icon(name, checked ? Tint::OnAccent : Tint::Text)); };
    apply(button->isChecked());
    QObject::connect(button, &QAbstractButton::toggled, button, apply);
}
