// This is the new StartNetwork function with fallback logic
void WifiBoard::StartNetwork() {
    // --- HARDCODED DIRECT CONNECTION WITH FALLBACK ---

    // Network 1 (Primary)
    const char* primary_ssid = "f-playground-2";
    const char* primary_password = "pythaaaa";

    // Network 2 (Fallback)
    const char* fallback_ssid = "f-playground-1";
    const char* fallback_password = "pythaaaa";

    // --- Connection Logic ---
    ESP_LOGI(TAG, "Starting network connection sequence...");

    // 1. Attempt to connect to the Primary network (15-second timeout)
    ESP_LOGI(TAG, "Trying primary network: %s", primary_ssid);
    WiFi.begin(primary_ssid, primary_password);
    for (int i = 0; i < 30; i++) { // 30 attempts * 500ms = 15 seconds
        if (WiFi.status() == WL_CONNECTED) {
            ESP_LOGI(TAG, "WiFi Connected to primary network! IP Address: %s", WiFi.localIP().toString().c_str());
            auto& application = Application::GetInstance();
            application.SetDeviceState(kDeviceStateNetworkReady);
            return; // Success, exit function
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 2. If primary failed, attempt to connect to the Fallback network (15-second timeout)
    ESP_LOGW(TAG, "Primary network failed. Trying fallback network: %s", fallback_ssid);
    WiFi.disconnect(true); // Disconnect from previous attempt
    vTaskDelay(pdMS_TO_TICKS(100));
    WiFi.begin(fallback_ssid, fallback_password);
    for (int i = 0; i < 30; i++) {
        if (WiFi.status() == WL_CONNECTED) {
            ESP_LOGI(TAG, "WiFi Connected to fallback network! IP Address: %s", WiFi.localIP().toString().c_str());
            auto& application = Application::GetInstance();
            application.SetDeviceState(kDeviceStateNetworkReady);
            return; // Success, exit function
        }
        TaskDelay(pdMS_TO_TICKS(500));
    }

    // 3. If both networks fail, log an error and reboot
    ESP_LOGE(TAG, "Failed to connect to any configured WiFi network. Rebooting.");
    esp_restart();
}