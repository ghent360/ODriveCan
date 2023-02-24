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
static ToggleMenuItem subMenu2("Sub Item 2", nullptr);
static ToggleMenuItem subMenu3("Sub Item 3", nullptr);
static SimpleMenuItem subMenu4("Sub Item 4", nullptr);
static MenuItem *subMenuItems[4] = {
  &subMenu1,
  &subMenu2,
  &subMenu3,
  &subMenu4,
};

static Menu subMenu(
  "Sub Menu 1", subMenuItems, __count_of(subMenuItems), 140, 30, 175, 178);

static SimpleMenuItem rootMenu1("Item 1", nullptr);
static ToggleMenuItem rootMenu2("Item 2", nullptr);
static ToggleMenuItem rootMenu3("Item 3", nullptr);
static ToggleMenuItem rootMenu4("Item 4", nullptr);
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

static Menu rootMenu(
  "", rootMenuItems, __count_of(rootMenuItems), 130, 30, 185, 178);

void SimpleMenuItem::draw() {
  if (!dirty_) return;
  tft.fillRect(
    x_, y_ + 1, w_, h_ - 2, selected_ ? ILI9341_YELLOW : ILI9341_BLACK);
  tft.setTextColor(selected_ ? ILI9341_DARKGREY : ILI9341_YELLOW);
  tft.setFont(Inconsolata_Regular_10);
  tft.drawString(title_, x_, y_ + 2);
  dirty_ = false;
}

void ToggleMenuItem::draw() {
  if (!dirty_) return;
  tft.fillRect(
    x_, y_ + 1, w_, h_ - 2, selected_ ? ILI9341_YELLOW : ILI9341_BLACK);
  tft.setTextColor(selected_ ? ILI9341_DARKGREY : ILI9341_YELLOW);
  tft.setFont(Inconsolata_Regular_10);
  tft.drawString(title_, x_, y_ + 2);
  tft.drawString(state_ ? "ON" : "OFF", x_ + w_ - 4 - 20, y_ + 2);
  dirty_ = false;
}

void Menu::init() {
  uint16_t y = y_ + 2;
  for(uint8_t idx = 0; idx < len_; idx++) {
    items_[idx]->init();
    items_[idx]->setMenuPos(x_ + 2, y, w_ - 4, menuItemHeight);
    y += menuItemHeight;
  }
  items_[selected_idx_]->setSelectState(true);
}

void Menu::draw() {
  if (!dirty()) return;
  if (is_open_) {
    if (dirty_) {
      tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);
      tft.drawRoundRect(x_, y_, w_, h_, 4, ILI9341_WHITE);
    }
    for(uint8_t idx = 0; idx < len_; idx++) {
      items_[idx]->draw();
    }
  } else {
    tft.fillRect(
      closed_x_,
      closed_y_ + 1,
      closed_w_,
      closed_h_ - 2,
      selected_ ? ILI9341_YELLOW : ILI9341_BLACK);
    tft.setTextColor(selected_ ? ILI9341_DARKGREY : ILI9341_YELLOW);
    tft.setFont(Inconsolata_Regular_10);
    tft.drawString(title_, closed_x_, closed_y_ + 2);
  }
  dirty_ = false;
}

bool Menu::dirty() const {
  if (dirty_) return true;
  if (is_open_) {
    for(uint8_t idx = 0; idx < len_; idx++) {
      if (items_[idx]->dirty()) return true;
    }
  }
  return false;
}

void Menu::select() {
  if (!is_open_) {
    remoteDisplay.controller().pushMenu(this);
    is_open_ = true;
    dirty_ = true;
  } else {
    items_[selected_idx_]->select();
  }
}

void Menu::back() {
  is_open_ = false;
  dirty_ = true;
  tft.fillRect(x_, y_, w_, h_, ILI9341_BLACK);
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
    m->redraw();
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
  if (!dirty()) return;
  if (menu_idx_ >= 0) {
    menu_stack_[menu_idx_]->draw();
  }
  dirty_ = false;
}
