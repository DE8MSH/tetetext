/*  ESP32 – NDR Teletext 100 / 652 / 656 / 658  (10s Rotation, Subpages)
    - Diff-Rendering (sanfter Repaint)
    - Subpages automatisch (…_01.htm bis …_NN.htm)
    - ISO-6937-nahe Mapping (Umlaute, Akzente -> ASCII)
    - SPIFFS-Cache je Seite/Sub
    - Exakte ETS 300 706 Level-1 Farbpalette
    - RGB-LED AUS (CYD)

    Kompatibel mit ESP32 Arduino Core 2.x **und** 3.x (LEDC-Wrapper)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <FS.h>
#include <SPIFFS.h>
#include <vector>
#include "cred.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// -------------------- RGB LED (CYD) --------------------
#define ledPinR 4
#define ledPinG 16
#define ledPinB 17
#define ledChnR 0
#define ledChnG 1
#define ledChnB 2
#define ledRes  8
#define ledFrq  5000

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_INIT_PIN(pin,freq,res)   do{ ledcAttach((pin),(freq),(res)); }while(0)
  #define LEDC_WRITE_PIN(pin,duty)      ledcWrite((pin),(duty))
#else
  static inline int _chFromPin(int pin){
    if(pin==ledPinR) return ledChnR;
    if(pin==ledPinG) return ledChnG;
    return ledChnB;
  }
  #define LEDC_INIT_PIN(pin,freq,res)   do{ int ch=_chFromPin((pin)); ledcSetup(ch,(freq),(res)); ledcAttachPin((pin),ch);}while(0)
  #define LEDC_WRITE_PIN(pin,duty)      do{ int ch=_chFromPin((pin)); ledcWrite(ch,(duty)); }while(0)
#endif
static void ledRGB(uint8_t R,uint8_t G,uint8_t B){ LEDC_WRITE_PIN(ledPinR,255-R); LEDC_WRITE_PIN(ledPinG,255-G); LEDC_WRITE_PIN(ledPinB,255-B); }
static void ledRGB_init(){ LEDC_INIT_PIN(ledPinR,ledFrq,ledRes); LEDC_INIT_PIN(ledPinG,ledFrq,ledRes); LEDC_INIT_PIN(ledPinB,ledFrq,ledRes); ledRGB(0,0,0); }

// -------------------- Seiten & Raster --------------------
static const int PAGES[] = {100, 108,109,110, 652, 656, 658, 669, 112, 590};
static const int NUM_PAGES = sizeof(PAGES)/sizeof(PAGES[0]);

static const int COLS = 40;
static const int ROWS = 24;

static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
static const int CELL_W = SCREEN_W / COLS; // 8
static const int CELL_H = SCREEN_H / ROWS; // 10

// -------------------- Display --------------------
#define USE_TFT_ESPI 1
#if USE_TFT_ESPI
  #include <TFT_eSPI.h>
  TFT_eSPI tft;
  #define INIT_TFT()           do{ tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);}while(0)
  #define COLOR565(r,g,b)      tft.color565((r),(g),(b))
  #define FILL_RECT(x,y,w,h,c) tft.fillRect((x),(y),(w),(h),(c))
  #define DRAW_CHAR(x,y,ch,fg,bg) tft.drawChar((x),(y),(uint8_t)(ch),(fg),(bg),1)
  #define FILL_SCREEN(c)       tft.fillScreen((c))
#else
  #include <cyd_tft.h>
  CydTft tft;
  #define INIT_TFT()           do{ tft.begin(); tft.setRotation(1); tft.fillScreen(COLOR_BLACK);}while(0)
  #define COLOR565(r,g,b)      tft.color565((r),(g),(b))
  #define FILL_RECT(x,y,w,h,c) tft.fillRect((x),(y),(w),(h),(c))
  #define DRAW_CHAR(x,y,ch,fg,bg) tft.drawChar((x),(y),(uint8_t)(ch),(fg),(bg),1)
  #define FILL_SCREEN(c)       tft.fillScreen((c))
#endif

// Backlight ggf. abschalten
static void backlightOffIfAvailable(){
#ifdef TFT_BL
  pinMode(TFT_BL,OUTPUT);
  digitalWrite(TFT_BL,LOW);
#endif
}

// -------------------- ETS 300 706 Level-1 Palette --------------------
static inline uint16_t make565(uint8_t r,uint8_t g,uint8_t b){ return COLOR565(r,g,b); }
static const uint16_t C_BLACK   = make565(0,0,0);
static const uint16_t C_RED     = make565(255,0,0);
static const uint16_t C_GREEN   = make565(0,255,0);
static const uint16_t C_YELLOW  = make565(255,255,0);
static const uint16_t C_BLUE    = make565(0,0,255);
static const uint16_t C_MAGENTA = make565(255,0,255);
static const uint16_t C_CYAN    = make565(0,255,255);
static const uint16_t C_WHITE   = make565(255,255,255);

static uint16_t colorFromIndex(int idx){
  switch(idx&7){
    case 0: return C_BLACK; case 1: return C_RED; case 2: return C_GREEN; case 3: return C_YELLOW;
    case 4: return C_BLUE;  case 5: return C_MAGENTA; case 6: return C_CYAN; default: return C_WHITE;
  }
}

// -------------------- Utils --------------------
static int hexNib(char ch){ if(ch>='0'&&ch<='9') return ch-'0'; if(ch>='a'&&ch<='f') return 10+ch-'a'; if(ch>='A'&&ch<='F') return 10+ch-'A'; return 0; }
static bool parseHexColor(const String &hex, uint16_t &out){
  if(hex.length()==7 && hex[0]=='#'){ int r=(hexNib(hex[1])<<4)|hexNib(hex[2]); int g=(hexNib(hex[3])<<4)|hexNib(hex[4]); int b=(hexNib(hex[5])<<4)|hexNib(hex[6]); out = make565(r,g,b); return true; }
  if(hex.length()==4 && hex[0]=='#'){ int r=hexNib(hex[1]); r=(r<<4)|r; int g=hexNib(hex[2]); g=(g<<4)|g; int b=hexNib(hex[3]); b=(b<<4)|b; out = make565(r,g,b); return true; }
  return false;
}

// -------------------- UTF-8 + Entity --------------------
static uint32_t utf8NextRaw(const String &s, size_t &i){
  if(i >= (size_t)s.length()) return 0;
  uint8_t c=(uint8_t)s[i++];
  if(c<0x80) return c;
  if((c>>5)==0x6){ if(i>=(size_t)s.length()) return 63; uint8_t c2=(uint8_t)s[i++]; return ((c&0x1F)<<6)|(c2&0x3F); }
  if((c>>4)==0xE){ if(i+1>(size_t)s.length()) return 63; uint8_t c2=(uint8_t)s[i++]; uint8_t c3=(uint8_t)s[i++]; return ((c&0x0F)<<12)|((c2&0x3F)<<6)|(c3&0x3F); }
  while(i<(size_t)s.length() && (((uint8_t)s[i])&0xC0)==0x80) i++;
  return 63;
}
static uint32_t nextCodepoint(const String &s, size_t &i){
  if(i >= (size_t)s.length()) return 0;
  if(s[i] != '&') return utf8NextRaw(s,i);
  int semi = s.indexOf(';',(int)i);
  if(semi < 0) return utf8NextRaw(s,i);
  String ent = s.substring((int)i+1, semi); i = (size_t)semi + 1; ent.trim();
  if(ent=="lt")return 60; if(ent=="gt")return 62; if(ent=="amp")return 38; if(ent=="quot")return 34; if(ent=="apos")return 39; if(ent=="nbsp")return 32;
  if(ent.length()>1 && ent[0]=='#'){
    bool hex = ent.length()>2 && (ent[1]=='x'||ent[1]=='X'); uint32_t v=0;
    for(size_t k=(hex?2:1); k<ent.length(); ++k){ char ch=ent[k]; v = hex ? (v<<4)|hexNib(ch) : v*10 + ((ch>='0'&&ch<='9')?(ch-'0'):0); }
    if(v>0x10FFFFu) v=0x20u; return v;
  }
  return 38;
}

// -------------------- ISO-6937-nahe ASCII-Fallback --------------------
static char iso6937_ascii(uint32_t cp){
  // häufige Teletext-Zeichen -> 7-bit Annäherung
  switch(cp){
    case 0x00E4: return 'a'; case 0x00F6: return 'o'; case 0x00FC: return 'u';
    case 0x00C4: return 'A'; case 0x00D6: return 'O'; case 0x00DC: return 'U';
    case 0x00DF: return 's'; // ß -> s (ein Zeichen)
    case 0x00E9: return 'e'; case 0x00E8: return 'e'; case 0x00EA: return 'e';
    case 0x00E0: return 'a'; case 0x00E1: return 'a'; case 0x00E2: return 'a';
    case 0x00F1: return 'n'; case 0x00E7: return 'c';
    default: return (cp<128u)?(char)cp:' ';
  }
}

// -------------------- Grid --------------------
struct Cell{ char ch; uint16_t fg; uint16_t bg; uint32_t cp; };
static Cell grid[ROWS][COLS];
static Cell prevGrid[ROWS][COLS];

static void clearGrid(Cell (&g)[ROWS][COLS]){ for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++) g[r][c]=Cell{(char)32,C_WHITE,C_BLACK,32u}; }

// -------------------- Styles (class/style) --------------------
static bool getAttr(const String &tag,const String &name,String &out){
  int p = tag.indexOf(name + "="); if(p<0) return false;
  int pos = p + name.length() + 1; int q1=-1,q2=-1;
  if(pos < tag.length() && (tag[pos]==34 || tag[pos]==39)){ char q=tag[pos]; q1=pos+1; q2=tag.indexOf((char)q,q1); }
  else{ q1=pos; q2=tag.indexOf(" ",q1); if(q2<0) q2=tag.length(); }
  if(q1>=0 && q2>q1){ out=tag.substring(q1,q2); return true; }
  return false;
}
static uint16_t colorFromClassToken(const String &tok){
  for(size_t i=0;i<tok.length();++i){ char ch=tok[i]; if(ch>='0'&&ch<='7') return colorFromIndex(ch-'0'); }
  return C_WHITE;
}
static void applyClasses(const String &classes,uint16_t &fg,uint16_t &bg){
  int s=0;
  while(s<classes.length()){
    int sp2=classes.indexOf(" ",s); String tok=(sp2<0)?classes.substring(s):classes.substring(s,sp2);
    tok.trim(); String tl=tok; tl.toLowerCase();
    if(tl.length()){
      if(tl[0]=='f') fg=colorFromClassToken(tl);
      if(tl.startsWith("b")) bg=colorFromClassToken(tl);
      if(tl.startsWith("bg")) bg=colorFromClassToken(tl);
    }
    if(sp2<0) break; s=sp2+1;
  }
}
static void applyStyleColors(const String &style,uint16_t &fg,uint16_t &bg){
  String s=style; s.toLowerCase();
  int p=s.indexOf("color:"); if(p>=0){ int h=s.indexOf('#',p); if(h>=0){ int e=s.indexOf(';',h); String hx=(e>h)?s.substring(h,e):s.substring(h); uint16_t c; if(parseHexColor(hx,c)) fg=c; } }
  int q=s.indexOf("background"); if(q>=0){ int h=s.indexOf('#',q); if(h>=0){ int e=s.indexOf(';',h); String hx=(e>h)?s.substring(h,e):s.substring(h); uint16_t c; if(parseHexColor(hx,c)) bg=c; } }
}

// -------------------- Parser --------------------
static void parsePreTxtToGrid(const String &preHtml){
  clearGrid(grid);
  uint16_t fg=C_WHITE, bg=C_BLACK; std::vector<uint16_t> fgSt; std::vector<uint16_t> bgSt;
  int row=0,col=0;

  for(size_t i=0;i<preHtml.length() && row<ROWS;){
    if(preHtml[i]=='<'){
      int close = preHtml.indexOf((char)62,(int)i+1); if(close<0) break;
      String tag=preHtml.substring((int)i+1,close); tag.trim();
      bool closing = tag.length()&&tag[0]=='/';
      if(closing){
        String tn=tag.substring(1); tn.trim(); tn.toLowerCase();
        if(tn.startsWith("span")||tn.startsWith("b")){
          if(!fgSt.empty()){ fg=fgSt.back(); fgSt.pop_back(); }
          if(!bgSt.empty()){ bg=bgSt.back(); bgSt.pop_back(); }
        }
      }else{
        String tn=tag; int sp=tn.indexOf(" "); if(sp>0) tn=tn.substring(0,sp); tn.toLowerCase();
        if(tn=="span"||tn=="b"){
          fgSt.push_back(fg); bgSt.push_back(bg);
          String classes; if(getAttr(tag,"class",classes)) applyClasses(classes,fg,bg);
          String style;   if(getAttr(tag,"style",style))   applyStyleColors(style,fg,bg);
        }
      }
      i=(size_t)close+1; continue;
    }

    uint32_t cp = nextCodepoint(preHtml,i);
    if(cp==10u){ while(col<COLS) grid[row][col++]=Cell{(char)32,fg,bg,32u}; row++; col=0; continue; }
    if(cp==160u) cp=32u;

    // NDR PUA
    if(cp>=0xE000u && cp<=0xF8FFu){
      if(col<COLS) grid[row][col++]=Cell{(char)32,fg,bg,cp}; // Renderer entscheidet
      continue;
    }

    char ch = iso6937_ascii(cp);
    if(col<COLS) grid[row][col++]=Cell{ch,fg,bg,cp};
  }

  // Rest leeren
  for(;row<ROWS;++row){ for(col=0;col<COLS;++col) grid[row][col]=Cell{(char)32,C_WHITE,C_BLACK,32u}; }
}

// -------------------- HTML <pre class="txt"> --------------------
static bool extractPreByClass(const String &html,const String &cls,String &out){
  String needle = String("<pre class=\"")+cls+"\"";
  int p=html.indexOf(needle); if(p<0) return false;
  int s=html.indexOf((char)62,p); if(s<0) return false; s++;
  int e=html.indexOf("</pre>",s); if(e<0) return false;
  out = html.substring(s,e); return true;
}

// -------------------- HTTP + Subpages --------------------
static bool httpGet(const String &url, String &body){
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http; http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.useHTTP10(true); http.setUserAgent("Mozilla/5.0");
  if(!http.begin(client,url)) return false;
  int code=http.GET();
  Serial.printf("GET %s -> %d\n", url.c_str(), code);
  if(code==200){ body=http.getString(); http.end(); return true; }
  http.end(); return false;
}

// Zähle Subpages anhand Links …_02.htm …_NN.htm (Fallback: mindestens 1)
static int detectSubpages(int page, const String &html){
  int maxSub=1;
  for(int s=2;s<=16;s++){ // Teletext typ. <= 16
    char buf[16]; snprintf(buf,sizeof(buf),"%d_%02d.htm",page,s);
    if(html.indexOf(buf)>=0) maxSub=s;
  }
  return maxSub;
}

static String makeUrl(int page,int sub){
  char subStr[3]; snprintf(subStr,sizeof(subStr),"%02d",sub);
  return String("https://www.ndr.de/public/teletext/")+page+"_"+subStr+".htm";
}

// -------------------- SPIFFS Cache --------------------
static String cachePath(int page,int sub){ char p[32]; snprintf(p,sizeof(p),"/tt_%d_%02d.pre",page,sub); return String(p); }
static void saveCache(int page,int sub,const String &pre){ File f=SPIFFS.open(cachePath(page,sub),FILE_WRITE,true); if(!f) return; f.print(pre); f.close(); }
static bool loadCache(int page,int sub,String &pre){ File f=SPIFFS.open(cachePath(page,sub),"r"); if(!f) return false; pre=f.readString(); f.close(); return pre.length(); }

// -------------------- Rendering-Helfer --------------------
static void drawNdrThinHLine(int x,int y,uint16_t fg){ int h=(CELL_H>=10)?2:1; int yy=y+(CELL_H-h)/2; FILL_RECT(x,yy,CELL_W,h,fg); }
static void drawMosaic2x3(int x,int y,uint32_t mask,uint16_t fg){
  int lw=CELL_W/2, rw=CELL_W-lw; int th=CELL_H/3, mh=CELL_H/3, bh=CELL_H-th-mh;
  if(mask&0x01) FILL_RECT(x,      y,         lw,th,fg);
  if(mask&0x02) FILL_RECT(x+lw,   y,         rw,th,fg);
  if(mask&0x04) FILL_RECT(x,      y+th,      lw,mh,fg);
  if(mask&0x08) FILL_RECT(x+lw,   y+th,      rw,mh,fg);
  if(mask&0x10) FILL_RECT(x,      y+th+mh,   lw,bh,fg);
  if(mask&0x20) FILL_RECT(x+lw,   y+th+mh,   rw,bh,fg);
}
static void fillUL(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x,y,hw,hh,c); }
static void fillUR(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x+CELL_W-hw,y,hw,hh,c); }
static void fillLL(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x,y+CELL_H-hh,hw,hh,c); }
static void fillLR(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x+CELL_W-hw,y+CELL_H-hh,hw,hh,c); }
static void drawBlockByCodepoint(int x,int y,uint32_t cp,uint16_t fg){
  int hw=(CELL_W+1)/2, hh=(CELL_H+1)/2;
  if(cp>=0x2581 && cp<=0x2588){ int eighths=(int)(cp-0x2580); int h=(CELL_H*eighths)/8; FILL_RECT(x,y+(CELL_H-h),CELL_W,h,fg); return; }
  if(cp==0x2594){ int h=(CELL_H+7)/8; FILL_RECT(x,y,CELL_W,h,fg); return; }
  if(cp==0x2580){ FILL_RECT(x,y,CELL_W,CELL_H/2,fg); return; }
  if(cp==0x2588){ FILL_RECT(x,y,CELL_W,CELL_H,fg); return; }
  if(cp>=0x2589 && cp<=0x258F){ int map[7]={7,6,5,4,3,2,1}; int idx=(int)(cp-0x2589); int w=(CELL_W*map[idx])/8; FILL_RECT(x,y,w,CELL_H,fg); return; }
  if(cp==0x2590){ FILL_RECT(x+CELL_W/2,y,CELL_W-CELL_W/2,CELL_H,fg); return; }
  if(cp==0x2595){ int w=(CELL_W+7)/8; FILL_RECT(x+CELL_W-w,y,w,CELL_H,fg); return; }
  switch(cp){
    case 0x2596: fillLL(x,y,hw,hh,fg); return;
    case 0x2597: fillLR(x,y,hw,hh,fg); return;
    case 0x2598: fillUL(x,y,hw,hh,fg); return;
    case 0x2599: fillUL(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return;
    case 0x259A: fillUL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return;
    case 0x259B: fillUL(x,y,hw,hh,fg); fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); return;
    case 0x259C: fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return;
    case 0x259D: fillUR(x,y,hw,hh,fg); return;
    case 0x259E: fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); return;
    case 0x259F: fillUR(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); fillUL(x,y,hw,hh,fg); return;
  }
}

// -------------------- Diff-Renderer --------------------
static void renderDiff(){
  for(int r=0;r<ROWS;r++){
    for(int c=0;c<COLS;c++){
      const Cell &cur=grid[r][c];
      Cell &old=prevGrid[r][c];
      if(cur.ch==old.ch && cur.fg==old.fg && cur.bg==old.bg && cur.cp==old.cp) continue; // unverändert

      int x=c*CELL_W, y=r*CELL_H;
      // BG nur neu malen, wenn sich BG/Art geändert hat (spart Flackern)
      if(cur.bg!=old.bg || (cur.cp!=old.cp)){
        FILL_RECT(x,y,CELL_W,CELL_H,cur.bg);
      }

      // PUA
      if(cur.cp>=0xE000u && cur.cp<=0xF8FFu){
        if(cur.cp==0xE00Cu){ drawNdrThinHLine(x,y,cur.fg); }
        else if(cur.cp<=0xE03Fu){ drawMosaic2x3(x,y,cur.cp-0xE000u,cur.fg); }
        else{ FILL_RECT(x,y,CELL_W,CELL_H,cur.fg); }
      }
      // Unicode Blockgrafik
      else if((cur.cp>=0x2580 && cur.cp<=0x259F) || cur.cp==0x2588){
        drawBlockByCodepoint(x,y,cur.cp,cur.fg);
      } else {
        int cx=x+((CELL_W-6)/2), cy=y+((CELL_H-8)/2);
        DRAW_CHAR(cx,cy,(uint8_t)cur.ch,cur.fg,cur.bg);
      }

      old = cur; // aktualisieren
    }
  }
}

// -------------------- Seite laden/anzeigen --------------------
static bool loadHtml(int page,int sub,String &html){
  String url=makeUrl(page,sub);
  if(httpGet(url,html)) return true;
  return false;
}

static bool showPage(int page,int sub){
  String html;
  bool online = loadHtml(page,sub,html);
  String pre;
  if(online){
    if(!extractPreByClass(html,"txt",pre)){ // Fallback: erstes <pre>
      int p=html.indexOf("<pre"); if(p>=0){ int q=html.indexOf((char)62,p); int e=html.indexOf("</pre>",q+1); if(q>0 && e>q) pre=html.substring(q+1,e); }
    }
    if(pre.length()) saveCache(page,sub,pre);
  }else{
    if(!loadCache(page,sub,pre)) return false; // weder online noch Cache
  }

  parsePreTxtToGrid(pre);
  renderDiff();
  return true;
}

// Ermittele Subpage-Anzahl (online, ggf. via Cache 01)
static int getSubCount(int page){
  String html;
  if(loadHtml(page,1,html)){
    int n = detectSubpages(page,html);
    // Cache pre von sub 1 direkt befüllen
    String pre; if(extractPreByClass(html,"txt",pre)) saveCache(page,1,pre);
    return n;
  }
  // Offline: versuche Caches 01..16
  int maxSub=0; String dummy;
  for(int s=1;s<=16;s++){ if(loadCache(page,s,dummy)) maxSub=s; }
  return maxSub>0?maxSub:1;
}

// -------------------- Setup / Loop --------------------
void setup(){
  Serial.begin(115200);
  SPIFFS.begin(true);
  ledRGB_init();
  backlightOffIfAvailable();
  INIT_TFT();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WLAN verbinden");
  unsigned long t0=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<10000){ delay(300); Serial.print('.'); }
  Serial.println(); Serial.printf("WLAN: %s\n", WiFi.status()==WL_CONNECTED?"ok":"offline");

  clearGrid(prevGrid); // Startzustand für Diff
  FILL_SCREEN(C_BLACK);
}

void loop(){
  for(int i=0;i<NUM_PAGES;i++){
    int page=PAGES[i];
    int subCount = getSubCount(page);
    if(subCount<1) subCount=1;
    for(int s=1;s<=subCount;s++){
      Serial.printf("Seite %d_%02d\n", page, s);
      if(!showPage(page,s)){
        // Fehleranzeige (ohne Flackern)
        int cx=8, cy=SCREEN_H/2-8;
        const char* msg="Offline / kein Cache";
        for(int k=0; msg[k]; ++k) DRAW_CHAR(cx+k*8,cy,(uint8_t)msg[k],C_RED,C_BLACK);
      }
      for(int t=0;t<10;t++) delay(1000);
    }
  }
}
