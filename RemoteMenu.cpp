/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteMenu.h"
#include "RemoteDisplay.h"
#include <ILI9341_t3n.h> // Hardware-specific library

#include "font_Inconsolata-Regular.h"

extern ILI9341_t3n tft;

static constexpr uint8_t menuItemHeight = 14;
static SimpleMenuItem rootMenu1("Item1", nullptr);
static MenuItem *rootMenuItems[1] = { &rootMenu1 };
static Menu rootMenu(
  "",
  rootMenuItems,
  sizeof(rootMenuItems) / sizeof(rootMenuItems[0]),
  80,
  20,
  235,
  128);

void SimpleMenuItem::init() {
  if (title_.length() > 0) {
    int16_t x1, y1;
    tft.setFont(Inconsolata_Regular_10);
    tft.getTextBounds(title_.c_str(), x_, y_, &x1, &y1, &w_, &h_);
  } else {
    w_ = 0;
    h_ = 0;
  }
}

void SimpleMenuItem::draw() {
  if (!dirty_) return;
  tft.fillRect(
    x_, y_, w_, h_, ILI9341_BLACK);
  if (title_.length() > 0) {
    tft.setFont(Inconsolata_Regular_10);
    tft.setTextColor(ILI9341_YELLOW);
    tft.drawString(title_, x_, y_);
  }
  dirty_ = false;
}

void Menu::init() {
  for(uint8_t idx = 0; idx < len_; idx++) {
    items_[idx]->init();
  }
}

void Menu::draw() {
  if (!dirty_) return;
  tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);
  if (is_open_) {
    tft.drawRoundRect(x_, y_, w_, h_, 4, ILI9341_WHITE);
    uint16_t x = x_ + 2;
    uint16_t y = y_ + 2;
    tft.setFont(Inconsolata_Regular_10);
    for(uint8_t idx = 0; idx < len_; idx++) {
      if (idx == selected_) {
        tft.fillRect(x, y, w_ - 2, menuItemHeight - 2, ILI9341_YELLOW);
      }
      tft.setTextColor(idx == selected_ ? ILI9341_DARKGREY : ILI9341_YELLOW);
      tft.drawString(items_[idx]->str(), x, y);
      y += menuItemHeight;
    }
  }
  dirty_ = false;
}

void Menu::select() {
  if (!is_open_) {
    is_open_ = true;
    dirty_ = true;
    menuController.pushMenu(this);
  } else {
    items_[selected_]->select();
  }
}

void Menu::back() {
  is_open_ = false;
  dirty_ = true;
  draw();
  menuController.popMenu();
}

void MenuController::select() {
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_]->select();
  } else {
    remoteDisplay.setMenuActive(true);
  }
}

void MenuController::pushMenu(Menu* m) {
  if (!m) return;
  if (menu_idx_ < maxNestedMenus - 1) {
    menu_idx_++;
    menu_stack_[menu_idx_] = m;
    m->init();
    dirty_ = true;
  }
}

void MenuController::popMenu() {
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_] = nullptr;
    menu_idx_--;
    if (menu_idx_ < 0) {
      remoteDisplay.setMenuActive(false);
    }
    dirty_ = true;
  }
}

MenuController menuController;
