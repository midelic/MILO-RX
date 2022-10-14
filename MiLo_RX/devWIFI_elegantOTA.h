
#if defined(ESP8266_PLATFORM) || defined(ESP32_PLATFORM)
	
	//#define HOME_WIFI_CREDENTIALS
	
	#ifdef HOME_WIFI_CREDENTIALS
		#define HOME_WIFI_SSID "yourSSID"
		#define HOME_WIFI_PASSWORD "yourpass"
	#endif
	
	#if defined(ESP32_PLATFORM)
		#include <WiFi.h>
		#include <ESPmDNS.h>
		#include <Update.h>
		#else
		#include <ESP8266WiFi.h>
		#include <ESP8266mDNS.h>
		#define wifi_mode_t WiFiMode_t
	#endif
	
	#include <DNSServer.h>
	#include <set>
	#include <StreamString.h>
	#include <ESPAsyncWebServer.h>
	#include <AsyncElegantOTA.h>
	//#include "web_index.h"
	#define GETCHAR *fmt
	void debugPrintf(const char* fmt, ...);
	#define DBGLN(...) 
	//#define DBGLN(msg, ...) debugPrintf(msg, ##__VA_ARGS__)
	#define DBGVLN(...)
	#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
	unsigned long rebootTime = 0;
	static bool wifiStarted = false;
	bool webserverPreventAutoStart = false;
	static wl_status_t laststatus = WL_IDLE_STATUS;
	volatile WiFiMode_t wifiMode = WIFI_OFF;
	static volatile WiFiMode_t changeMode = WIFI_OFF;
	static volatile unsigned long changeTime = 0;
	static const byte DNS_PORT = 53;
	static IPAddress netMsk(255, 255, 255, 0);
	static DNSServer dnsServer;
	static IPAddress ipAddress;
	static AsyncWebServer server(80);
	static bool servicesStarted = false;
	
	#define QUOTE(arg) #arg
	#define STR(macro) QUOTE(macro)
	const char *wifi_hostname = "milo_rx";
	const char *wifi_ap_ssid = "MiLo_RX";
	const char *wifi_ap_password = "milo_sx1280";//min 8 char
	const char *wifi_ap_address = "10.0.0.1";
	#ifdef HOME_WIFI_CREDENTIALS
		char home_wifi_ssid[33] = {HOME_WIFI_SSID};
		char home_wifi_password[65] = {HOME_WIFI_PASSWORD};
		#else
		char home_wifi_ssid[33] = {};
		char home_wifi_password[65] ={};
	#endif
	static char station_ssid[33];
	static char station_password[65];
	uint8_t  wifiUpdate = 1;
	uint8_t connectionState = 0;
	
	
	/** Is this an IP? */
	static boolean isIp(String str)
	{
		for (size_t i = 0; i < str.length(); i++)
		{
			int c = str.charAt(i);
			if (c != '.' && (c < '0' || c > '9'))
			{
				return false;
			}
		}
		return true;
	}
	
	/** IP to String? */
	static String toStringIp(IPAddress ip)
	{
		String res = "";
		for (int i = 0; i < 3; i++)
		{
			res += String((ip >> (8 * i)) & 0xFF) + ".";
		}
		res += String(((ip >> 8 * 3)) & 0xFF);
		return res;
	}
	
	static bool captivePortal(AsyncWebServerRequest *request)
	{
		//extern const char *wifi_hostname;
		
		if (!isIp(request->host()) && request->host() != (String(wifi_hostname) + ".local"))
		{
			DBGLN("Request redirected to captive portal");
			request->redirect(String("http://") + toStringIp(request->client()->localIP())+String("/update"));
			return true;
		}
		return false;
	}
	
	static void HandleReboot(AsyncWebServerRequest *request)
	{
		AsyncWebServerResponse *response = request->beginResponse(200, "application/json", "Kill -9, no more CPU time!");
		response->addHeader("Connection", "close");
		request->send(response);
		request->client()->close();
		rebootTime = millis() + 100;
	}
	/*
	static void WebUpdateSendNetworks(AsyncWebServerRequest *request)  //not used yet
	{
		int numNetworks = WiFi.scanComplete();
		if (numNetworks >= 0) {
			DBGLN("Found %d networks", numNetworks);
			std::set<String> vs;
			String s="[";
			for(int i=0 ; i<numNetworks ; i++) {
				String w = WiFi.SSID(i);
				DBGLN("found %s", w.c_str());
				if (vs.find(w)==vs.end() && w.length()>0) {
					if (!vs.empty()) s += ",";
					s += "\"" + w + "\"";
					vs.insert(w);
				}
			}
			s+="]";
			request->send(200, "application/json", s);
			} else {
			request->send(204, "application/json", "[]");
		}
	}
	*/
	static void sendResponse(AsyncWebServerRequest *request, const String &msg, WiFiMode_t mode) {
		AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", msg);
		response->addHeader("Connection", "close");
		request->send(response);
		request->client()->close();
		changeTime = millis();
		changeMode = mode;
	}
	
	static void WebUpdateAccessPoint(AsyncWebServerRequest *request)
	{
		DBGLN("Starting Access Point");
		String msg = String("Access Point starting, please connect to access point '") + wifi_ap_ssid + "' with password '" + wifi_ap_password + "'";
		sendResponse(request, msg, WIFI_AP);
	}
	
	static void WebUpdateConnect(AsyncWebServerRequest *request)
	{
		DBGLN("Connecting to home network");
		String msg = String("Connecting to network '") + station_ssid + "', connect to http://" +
		wifi_hostname + ".local from a browser on that network";
		sendResponse(request, msg, WIFI_STA);
	}
	
	static void WebUpdateSetHome(AsyncWebServerRequest *request)
	{
		String ssid = request->arg("network");
		String password = request->arg("password");
		
		DBGLN("Setting home network %s", ssid.c_str());
		strcpy(station_ssid, ssid.c_str());
		strcpy(station_password, password.c_str());
		WebUpdateConnect(request);
	}
	
	static void WebUpdateForget(AsyncWebServerRequest *request)
	{
		DBGLN("Forget home network");
		// If we have a flashed wifi network then let's try reconnecting to that otherwise start an access point
		if (home_wifi_ssid[0] != 0) {
			strcpy(station_ssid, home_wifi_ssid);
			strcpy(station_password, home_wifi_password);
			String msg = String("Temporary network forgotten, attempting to connect to network '") + station_ssid + "'";
			sendResponse(request, msg, WIFI_STA);
		}
		else {
			station_ssid[0] = 0;
			station_password[0] = 0;
			String msg = String("Home network forgotten, please connect to access point '") + wifi_ap_ssid + "' with password '" + wifi_ap_password + "'";
			sendResponse(request, msg, WIFI_AP);
		}
	}
	
	static void WebUpdateHandleNotFound(AsyncWebServerRequest *request)
	{
		if (captivePortal(request))
		{ // If captive portal redirect instead of displaying the error page.
			return;
		}
		String message = F("File Not Found\n\n");
		message += F("URI: ");
		message += request->url();
		message += F("\nMethod: ");
		message += (request->method() == HTTP_GET) ? "GET" : "POST";
		message += F("\nArguments: ");
		message += request->args();
		message += F("\n");
		
		for (uint8_t i = 0; i < request->args(); i++)
		{
			message += String(F(" ")) + request->argName(i) + F(": ") + request->arg(i) + F("\n");
		}
		AsyncWebServerResponse *response = request->beginResponse(404, "text/plain", message);
		response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
		response->addHeader("Pragma", "no-cache");
		response->addHeader("Expires", "-1");
		request->send(response);
	}
	
	void wifiOff()
	{
		wifiStarted = false;
		WiFi.disconnect(true);
		WiFi.mode(WIFI_OFF);
		#if defined(ESP8266_PLATFORM)
			WiFi.forceSleepBegin();
		#endif
	}
	
	static void startWiFi(unsigned long now)
	{
		if (wifiStarted) {
			return;
		}
		//here before stop the radio
		connectionState = wifiUpdate; 
		DBGLN("Begin Webupdater");
		
		WiFi.persistent(false);
		WiFi.disconnect();
		WiFi.mode(WIFI_OFF);
		#if defined(ESP8266_PLATFORM)
			WiFi.setOutputPower(13);
			WiFi.setPhyMode(WIFI_PHY_MODE_11N);
			#elif defined(ESP32_PLATFORM)
			WiFi.setTxPower(WIFI_POWER_13dBm);
		#endif
		if (home_wifi_ssid[0] != 0) {
			strcpy(station_ssid, home_wifi_ssid);
			strcpy(station_password, home_wifi_password);
		}
		if (station_ssid[0] == 0) {
			changeTime = now;
			changeMode = WIFI_AP;
		}
		else {
			changeTime = now;
			changeMode = WIFI_STA;
		}
		laststatus = WL_DISCONNECTED;
		wifiStarted = true;
	}
	
	static void startServices()
	{
		if (servicesStarted) {
			#if defined(ESP32_PLATFORM)
				//MDNS.end();
				// startMDNS();
			#endif
			return;
		}
		
		//server.on("/networks.json", WebUpdateSendNetworks);
		server.on("/sethome", WebUpdateSetHome);
		server.on("/forget", WebUpdateForget);
		server.on("/connect", WebUpdateConnect);
		server.on("/access", WebUpdateAccessPoint);
		server.on("/reboot", HandleReboot);
		server.onNotFound(WebUpdateHandleNotFound);
		
		server.begin();
		dnsServer.start(DNS_PORT, "*", ipAddress);
		dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
		servicesStarted = true;
		DBGLN("HTTPUpdateServer ready! Open http://%s.local in your browser", wifi_hostname);
	}
	
	static void HandleWebUpdate()
	{
		static bool scanComplete = false;
		unsigned long now = millis();
		wl_status_t status = WiFi.status();
		
		if (status != laststatus && wifiMode == WIFI_STA) {
			DBGLN("WiFi status %d", status);
			switch(status) {
				case WL_NO_SSID_AVAIL:
				case WL_CONNECT_FAILED:
				case WL_CONNECTION_LOST:
				changeTime = now;
				changeMode = WIFI_AP;
				break;
				case WL_DISCONNECTED: // try reconnection
				changeTime = now;
				break;
				default:
				break;
			}
			laststatus = status;
		}
		if (status != WL_CONNECTED && wifiMode == WIFI_STA && (now - changeTime) > 30000) {
			changeTime = now;
			changeMode = WIFI_AP;
			DBGLN("Connection failed %d", status);
		}
		if (changeMode != wifiMode && changeMode != WIFI_OFF && (now - changeTime) > 500) {
			switch(changeMode) {
				case WIFI_AP:
				DBGLN("Changing to AP mode");
				WiFi.disconnect();
				wifiMode = WIFI_AP;
				WiFi.mode(wifiMode);
				changeTime = now;
				WiFi.softAPConfig(ipAddress, ipAddress, netMsk);
				WiFi.softAP(wifi_ap_ssid, wifi_ap_password);
				#if defined(ESP8266_PLATFORM)
					scanComplete = false;
					WiFi.scanNetworksAsync([](int){
						scanComplete = true;
					});
					#else
					WiFi.scanNetworks(true);
				#endif
				startServices();
				break;
				case WIFI_STA:
				DBGLN("Connecting to home network '%s'", station_ssid);
				wifiMode = WIFI_STA;
				WiFi.mode(wifiMode);
				WiFi.setHostname(wifi_hostname); // hostname must be set after the mode is set to STA
				changeTime = now;
				WiFi.begin(station_ssid, station_password);
				startServices();
				default:
				break;
			}
			#if defined(ESP8266_PLATFORM)
				MDNS.notifyAPChange();
			#endif
			changeMode = WIFI_OFF;
		}
		
		#if defined(ESP8266_PLATFORM)
			if (scanComplete)
			{
				WiFi.mode(wifiMode);
				scanComplete = false;
			}
		#endif
		
		if (servicesStarted)
		{
			dnsServer.processNextRequest();
			#if defined(ESP8266_PLATFORM)
				MDNS.update();
			#endif
			// When in STA mode, a small delay reduces power use from 90mA to 30mA when idle
			// In AP mode, it doesn't seem to make a measurable difference, but does not hurt
			if (!Update.isRunning())
			delay(1);
		}
	}
	
	void WIFI_start()//in setup
	{
		wifiOff();//initialize wifi
		ipAddress.fromString(wifi_ap_address);
		AsyncElegantOTA.begin(&server);// Start AsyncElegantOTA
		connectionState = wifiUpdate;
	}
	
	void WIFI_event()//in loop
	{
		if (connectionState == wifiUpdate )
		{
			if (!wifiStarted) {
				startWiFi(millis());
			}
		}
		if (wifiStarted)
		{
			HandleWebUpdate();
		} 
	}
	
	void debugPrintf(const char* fmt, ...)
	{
		char c;
		const char *v;
		char buf[11];
		va_list  vlist;
		va_start(vlist,fmt);
		
		c = GETCHAR;
		while(c) {
			if (c == '%') {
				fmt++;
				c = GETCHAR;
				v = buf;
				buf[0] = 0;
				switch (c) {
					case 's':
					v = va_arg(vlist, const char *);
					break;
					case 'd':
					itoa(va_arg(vlist, int32_t), buf, DEC);
					break;
					case 'u':
					utoa(va_arg(vlist, uint32_t), buf, DEC);
					break;
					case 'x':
					utoa(va_arg(vlist, uint32_t), buf, HEX);
					break;
					default:
					break;
				}
				Serial.write((uint8_t*)v, strlen(v));
				} else {
				Serial.write(c);
			}
			fmt++;
			c = GETCHAR;
		}
		va_end(vlist);
	}
	
	
#endif
