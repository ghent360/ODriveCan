/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */

#include "RemoteMenu.h"
#include "RemoteDisplay.h"
#include "RemoteProtocol.h"
#include "util.h"
#include <ILI9341_t3n.h> // Hardware-specific library

#include "font_Inconsolata-Regular.h"

extern ILI9341_t3n tft;
extern CommandCodes command;

static constexpr uint8_t menuItemHeight = 14;

static SimpleMenuItem footMenuFL(
  "Front Left",
  [](SimpleMenuItem*) {
    command = CMD_MOVE_FL_FOOT;
  });

static SimpleMenuItem footMenuFR(
  "Front Right",
  [](SimpleMenuItem*) {
    command = CMD_MOVE_FR_FOOT;
  });

static SimpleMenuItem footMenuBL(
  "Back Left",
  [](SimpleMenuItem*) {
    command = CMD_MOVE_BL_FOOT;
  });

static SimpleMenuItem footMenuBR(
  "Back Right",
  [](SimpleMenuItem*) {
    command = CMD_MOVE_BR_FOOT;
  });

static MenuItem *footMenuItems[4] = {
  &footMenuFL,
  &footMenuFR,
  &footMenuBL,
  &footMenuBR,
};

static Menu footMenu(
  "Move foot",
  footMenuItems,
  __count_of(footMenuItems),
  150,
  30,
  185,
  178,
  [] (Menu*) { command = CMD_MOVE_FOOT_DONE; });

static SimpleMenuItem gainMenuHip(
  "Hip gain",
  [](SimpleMenuItem*) {
    command = CMD_EDIT_HIP_GAIN;
  });

static SimpleMenuItem gainMenuTie(
  "Tie gain",
  [](SimpleMenuItem*) {
    command = CMD_EDIT_TIE_GAIN;
  });

static SimpleMenuItem gainMenuShin(
  "Shin gain",
  [](SimpleMenuItem*) {
    command = CMD_EDIT_SHIN_GAIN;
  });

static MenuItem *gainMenuItems[3] = {
  &gainMenuHip,
  &gainMenuTie,
  &gainMenuShin,
};

static Menu gainMenu(
  "Edit gains",
  gainMenuItems,
  __count_of(gainMenuItems),
  150,
  30,
  185,
  178,
  [] (Menu*) { command = CMD_EDIT_GAIN_DONE; });

static SimpleMenuItem rootMenuClearErrors(
  "Clear Errors",
  [](SimpleMenuItem*) {
    command = CMD_CLEAR_ERRORS;
  });

static SimpleMenuItem rootMenuSetGains(
  "Modify Gains",
  [](SimpleMenuItem*) {
    command = CMD_SET_GAINS;
  });

static MenuItem *rootMenuItems[4] = {
  &rootMenuClearErrors,
  &rootMenuSetGains,
  &footMenu,
  &gainMenu,
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
  if (closeCb_) {
    closeCb_(this);
  }
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
