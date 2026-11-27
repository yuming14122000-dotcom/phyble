# PhyBLE 燒錄站（可自行架設版）

用 Chrome / Edge 的 Web Serial 燒錄 ESP32，同時寫入組別（與起始感測器）。
韌體清單放在 `firmware/manifest.json`，要新增感測器韌體只要放進 .bin 檔、在清單加一段即可，不用改網頁程式。

```
index.html                 燒錄網頁
js/esptool.bundle.js       esptool-js 0.7.1（燒錄引擎）
js/nvs.js                  產生 NVS 分區（組別、模式）
firmware/manifest.json     韌體清單
firmware/phyble-multi/     多感測器韌體（bootloader、分區表、boot_app0、app 與原始碼）
```

## 一、發佈到 GitHub Pages（免費，約 5 分鐘）

Web Serial 只能在 **https** 網址使用，GitHub Pages 剛好符合。

1. 登入 GitHub → 右上「+」→ **New repository**，名稱例如 `phyble`，選 **Public** → Create。
2. 在新倉庫頁面按 **uploading an existing file**，把這個資料夾裡的**全部檔案與資料夾**拖進去 → Commit changes。
3. 倉庫的 **Settings → Pages**：Source 選 **Deploy from a branch**，Branch 選 `main`、資料夾 `/ (root)` → Save。
4. 等 1～2 分鐘，網址會是 `https://你的帳號.github.io/phyble/`。用 Chrome 或 Edge 開啟即可燒錄。

> 也可以用 Netlify、Cloudflare Pages 或學校的 https 網站，只要整個資料夾原樣上傳。
> 直接雙擊 index.html 開啟會讀不到韌體清單；本機測試請在資料夾內執行 `python -m http.server`，再開 `http://localhost:8000`。

## 二、新增一份感測器韌體

### 1. 在 Arduino IDE 產生 .bin

- 開發板：**esp32 by Espressif 2.0.17**，板子選 **ESP32 Dev Module**
- 分區：**Huge APP (3MB No OTA/1MB SPIFFS)**（要和現有韌體一樣，組別才會留在同一個位置）
- 函式庫：**phyphox BLE 1.3.1**（新版 2.x 需要 3.x 以上的開發板套件）
- 選單「草稿碼 → 匯出已編譯的二進位檔」，在草稿碼資料夾的 `build/esp32.esp32.esp32/` 會得到：
  - `xxx.ino.bootloader.bin`、`xxx.ino.partitions.bin`、`xxx.ino.bin`（另有 `xxx.ino.merged.bin` 合併檔）

### 2. 上傳檔案

在 `firmware/` 下建一個新資料夾，例如 `firmware/light-sensor/`，放入上面的檔案。
`boot_app0.bin` 可以直接複製 `firmware/phyble-multi/boot_app0.bin`。

### 3. 在 manifest.json 加一段

```json
{
  "id": "light-sensor",
  "name": "光敏電阻 光強度",
  "version": "2026-12-01",
  "description": "光強度對時間",
  "nvs": { "addr": "0x9000", "size": "0x5000", "namespace": "phyble", "group": "group" },
  "parts": [
    { "addr": "0x1000",  "file": "light-sensor/light.ino.bootloader.bin", "name": "bootloader" },
    { "addr": "0x8000",  "file": "light-sensor/light.ino.partitions.bin", "name": "分區表" },
    { "addr": "0xe000",  "file": "light-sensor/boot_app0.bin",           "name": "boot_app0" },
    { "addr": "0x10000", "file": "light-sensor/light.ino.bin",           "name": "應用程式" }
  ]
}
```

加在 `"firmwares": [ ... ]` 裡，和前一段之間用逗號隔開。重新整理燒錄網頁，下拉選單就會出現。

- 用合併檔也可以：`"parts": [{ "addr": "0x0", "file": "light-sensor/light.ino.merged.bin", "name": "合併檔" }]`
- `nvs`：韌體有用 Preferences 讀 `phyble` / `group` 才需要；沒有的話整段刪掉，網頁就不會寫組別。
- `modes`（選填）：韌體有多種模式時，列出模式名稱與接線，網頁會顯示「起始感測器」按鈕與接線表，並寫入 `nvs.mode` 指定的 key。格式可參考 phyble-multi 那一段。

### 4. 讓新韌體也有組別名稱（建議）

在新程式的 `setup()` 開頭照這樣讀組別，藍牙名稱就會是 PHY-G01、PHY-G02…，並印出燒錄網頁用來確認的那一行：

```cpp
#include <Preferences.h>
Preferences prefs;
prefs.begin("phyble", true);
uint8_t groupNo = prefs.getUChar("group", 0);
prefs.end();
char bleName[16];
snprintf(bleName, sizeof(bleName), "PHY-G%02u", groupNo);
PhyphoxBLE::start(bleName);
Serial.printf("PHYBLE GROUP=%02u NAME=%s\n", groupNo, bleName);
```

## 三、臨時測試

網頁上「載入自己的 .bin」可以直接選電腦裡的 `.ino.merged.bin`（從 0x0 寫入）或 `.ino.bin`（寫入 0x10000），不用先上傳到網站。
