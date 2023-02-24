/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteMenu.h"
#include "RemoteDisplay.h"
#include "util.h"
#include <ILI9341_t3n.h> // Hardware-specific library

#include "font_Inconsolata-Regular.h"

extern ILI9341_t3n tft;

static constexpr uint8_t menuItemHeight = 14;
static SimpleMenuItem subMenu1("Sub Item 1", nullptr);
static SimpleMenuItem subMenu2("Sub Item 2", nullptr);
static SimpleMenuItem subMenu3("Sub Item 3", nullptr);
static SimpleMenuItem subMenu4("Sub Item 4", nullptr);
static MenuItem *subMenuItems[4] = {
  &subMenu1,
  &subMenu2,
  &subMenu3,
  &subMenu4,
};

static Menu subMenu("Sub Menu 1", subMenuItems, __count_of(subMenuItems), 140, 30, 175, 178);

static SimpleMenuItem rootMenu1("Item 1", nullptr);
static SimpleMenuItem rootMenu2("Item 2", nullptr);
static SimpleMenuItem rootMenu3("Item 3", nullptr);
static SimpleMenuItem rootMenu4("Item 4", nullptr);
static SimpleMenuItem rootMenu5("Item 5", nullptr);
static SimpleMenuItem rootMenu6("Item 6", nullptr);
static SimpleMenuItem rootMenu7("Item 7", nullptr);
static MenuItem *rootMenuItems[8] = {
  &rootMenu1,
  &rootMenu2,
  &rootMenu3,
  &rootMenu4,
  &rootMenu5,
  &subMenu,
  &rootMenu6,
  &rootMenu7,
};

static Menu rootMenu("", rootMenuItems, __count_of(rootMenuItems), 130, 30, 185, 178);

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
  Serial.println("menu::draw");
  if (is_open_) {
    Serial.println("menu::draw::is_open");
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
    remoteDisplay.controller().pushMenu(this);
    is_open_ = true;
    dirty_ = true;
  } else {
    items_[selected_]->select();
  }
}

void Menu::back() {
  is_open_ = false;
  dirty_ = true;
  draw();
  remoteDisplay.controller().popMenu();
}

void MenuController::init() {
  rootMenu.init();
}

void MenuController::select() {
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_]->select();
  } else {
    remoteDisplay.setMenuActive(true);
    rootMenu.select();
  }
}

void MenuController::back()  {
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_]->back();
    dirty_ = true;
  }
}

void MenuController::pushMenu(Menu* m) {
  if (!m) return;
  if (menu_idx_ < (maxNestedMenus - 1)) {
    menu_idx_++;
    menu_stack_[menu_idx_] = m;
    dirty_ = true;
  }
}

void MenuController::popMenu() {
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_] = nullptr;
    menu_idx_--;
    if (menu_idx_ < 0) {
      remoteDisplay.setMenuActive(false);
    } else {
      menu_stack_[menu_idx_]->redraw();
    }
    dirty_ = true;
  }
}

void MenuController::draw() {
  if (!dirty_) return;
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_]->draw();
  }
  dirty_ = false;
}
