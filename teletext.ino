/*  ESP32 – NDR Teletext (100/652/656/658 + Subpages)
    - Diff-Rendering, SPIFFS-Cache, ISO-6937-ASCII
    - PUA: E000..E03F contiguous, E040..E07F separated
    - CYD-RGB-LED aus
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

// ---------- CYD RGB-LED aus ----------
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
  static inline int _chFromPin(int pin){ if(pin==ledPinR) return ledChnR; if(pin==ledPinG) return ledChnG; return ledChnB; }
  #define LEDC_INIT_PIN(pin,freq,res)   do{ int ch=_chFromPin((pin)); ledcSetup(ch,(freq),(res)); ledcAttachPin((pin),ch);}while(0)
  #define LEDC_WRITE_PIN(pin,duty)      do{ int ch=_chFromPin((pin)); ledcWrite(ch,(duty)); }while(0)
#endif
static void ledRGB(uint8_t R,uint8_t G,uint8_t B){ LEDC_WRITE_PIN(ledPinR,255-R); LEDC_WRITE_PIN(ledPinG,255-G); LEDC_WRITE_PIN(ledPinB,255-B); }
static void ledRGB_init(){ LEDC_INIT_PIN(ledPinR,ledFrq,ledRes); LEDC_INIT_PIN(ledPinG,ledFrq,ledRes); LEDC_INIT_PIN(ledPinB,ledFrq,ledRes); ledRGB(0,0,0); }

// ---------- Seiten & Raster ----------
static const int PAGES[] = {100, 108,109,110, 652, 656, 658,665, 669, 112, 590};
static const int NUM_PAGES = sizeof(PAGES)/sizeof(PAGES[0]);
static const int COLS=40, ROWS=24;
static const int SCREEN_W=320, SCREEN_H=240;
static const int CELL_W=SCREEN_W/COLS;  // 8
static const int CELL_H=SCREEN_H/ROWS;  // 10

// ---------- Display ----------
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

static void backlightOffIfAvailable(){
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);
#endif
}

// ---------- Teletext-Palette ----------
static inline uint16_t make565(uint8_t r,uint8_t g,uint8_t b){ return COLOR565(r,g,b); }
static const uint16_t C_K=make565(0,0,0), C_R=make565(255,0,0), C_G=make565(0,255,0), C_Y=make565(255,255,0),
                      C_B=make565(0,0,255), C_M=make565(255,0,255), C_C=make565(0,255,255), C_W=make565(255,255,255);
static uint16_t colorFromIndex(int i){ switch(i&7){case 0:return C_K;case 1:return C_R;case 2:return C_G;case 3:return C_Y;case 4:return C_B;case 5:return C_M;case 6:return C_C;default:return C_W;} }

// ---------- Helpers ----------
static int hexNib(char ch){ if(ch>='0'&&ch<='9') return ch-'0'; if(ch>='a'&&ch<='f') return 10+ch-'a'; if(ch>='A'&&ch<='F') return 10+ch-'A'; return 0; }
static bool parseHexColor(const String &hex,uint16_t &out){
  if(hex.length()==7 && hex[0]=='#'){ int r=(hexNib(hex[1])<<4)|hexNib(hex[2]); int g=(hexNib(hex[3])<<4)|hexNib(hex[4]); int b=(hexNib(hex[5])<<4)|hexNib(hex[6]); out=make565(r,g,b); return true; }
  if(hex.length()==4 && hex[0]=='#'){ int r=hexNib(hex[1]); r=(r<<4)|r; int g=hexNib(hex[2]); g=(g<<4)|g; int b=hexNib(hex[3]); b=(b<<4)|b; out=make565(r,g,b); return true; }
  return false;
}

// ---------- UTF-8 + Entities ----------
static uint32_t utf8NextRaw(const String &s,size_t &i){
  if(i>=(size_t)s.length()) return 0;
  uint8_t c=(uint8_t)s[i++]; if(c<0x80) return c;
  if((c>>5)==0x6){ if(i>=(size_t)s.length())return 63; uint8_t c2=(uint8_t)s[i++]; return ((c&0x1F)<<6)|(c2&0x3F); }
  if((c>>4)==0xE){ if(i+1>(size_t)s.length())return 63; uint8_t c2=(uint8_t)s[i++], c3=(uint8_t)s[i++]; return ((c&0x0F)<<12)|((c2&0x3F)<<6)|(c3&0x3F); }
  while(i<(size_t)s.length() && (((uint8_t)s[i])&0xC0)==0x80) i++; return 63;
}
static uint32_t nextCodepoint(const String &s,size_t &i){
  if(i>=(size_t)s.length()) return 0;
  if(s[i]!='&') return utf8NextRaw(s,i);
  int semi=s.indexOf(';',(int)i); if(semi<0) return utf8NextRaw(s,i);
  String ent=s.substring((int)i+1,semi); i=(size_t)semi+1; ent.trim();
  if(ent=="lt")return 60; if(ent=="gt")return 62; if(ent=="amp")return 38; if(ent=="quot")return 34; if(ent=="apos")return 39; if(ent=="nbsp")return 32;
  if(ent.length()>1 && ent[0]=='#'){ bool hex=(ent.length()>2&&(ent[1]=='x'||ent[1]=='X')); uint32_t v=0;
    for(size_t k=(hex?2:1); k<ent.length(); ++k){ char ch=ent[k]; v = hex ? (v<<4)|hexNib(ch) : v*10 + (ch>='0'&&ch<='9'?ch-'0':0); }
    return (v>0x10FFFFu)?32u:v;
  }
  return 38;
}

// ---------- ISO-6937 → ASCII-Fallback ----------
static char iso6937_ascii(uint32_t cp){
  switch(cp){
    case 0x00E4: return 'a'; case 0x00F6: return 'o'; case 0x00FC: return 'u';
    case 0x00C4: return 'A'; case 0x00D6: return 'O'; case 0x00DC: return 'U';
    case 0x00DF: return 's';
    case 0x00E9: case 0x00E8: case 0x00EA: return 'e';
    case 0x00E0: case 0x00E1: case 0x00E2: return 'a';
    case 0x00F1: return 'n'; case 0x00E7: return 'c';
    default: return (cp<128u)?(char)cp:' ';
  }
}

// ---------- Grid ----------
struct Cell{ char ch; uint16_t fg,bg; uint32_t cp; };
static Cell grid[ROWS][COLS], prevGrid[ROWS][COLS];
static void clearGrid(Cell (&g)[ROWS][COLS]){ for(int r=0;r<ROWS;r++) for(int c=0;c<COLS;c++) g[r][c]=Cell{(char)32,C_W,C_K,32u}; }

// ---------- Styles ----------
static bool getAttr(const String &tag,const String &name,String &out){
  int p=tag.indexOf(name + "="); if(p<0) return false;
  int pos=p+name.length()+1, q1=-1,q2=-1;
  if(pos<tag.length() && (tag[pos]==34||tag[pos]==39)){ char q=tag[pos]; q1=pos+1; q2=tag.indexOf((char)q,q1); }
  else{ q1=pos; q2=tag.indexOf(" ",q1); if(q2<0) q2=tag.length(); }
  if(q1>=0 && q2>q1){ out=tag.substring(q1,q2); return true; }
  return false;
}
static uint16_t colorFromClassToken(const String &tok){ for(size_t i=0;i<tok.length();++i){ char ch=tok[i]; if(ch>='0'&&ch<='7') return colorFromIndex(ch-'0'); } return C_W; }
static void applyClasses(const String &classes,uint16_t &fg,uint16_t &bg){
  int s=0; while(s<classes.length()){
    int sp=classes.indexOf(" ",s); String tok=(sp<0)?classes.substring(s):classes.substring(s,sp); tok.trim(); String t=tok; t.toLowerCase();
    if(t.length()){ if(t[0]=='f') fg=colorFromClassToken(t); if(t.startsWith("b")) bg=colorFromClassToken(t); if(t.startsWith("bg")) bg=colorFromClassToken(t); }
    if(sp<0) break; s=sp+1;
  }
}
static void applyStyleColors(const String &style,uint16_t &fg,uint16_t &bg){
  String s=style; s.toLowerCase();
  int p=s.indexOf("color:"); if(p>=0){ int h=s.indexOf('#',p); if(h>=0){ int e=s.indexOf(';',h); String hx=(e>h)?s.substring(h,e):s.substring(h); uint16_t c; if(parseHexColor(hx,c)) fg=c; } }
  int q=s.indexOf("background"); if(q>=0){ int h=s.indexOf('#',q); if(h>=0){ int e=s.indexOf(';',h); String hx=(e>h)?s.substring(h,e):s.substring(h); uint16_t c; if(parseHexColor(hx,c)) bg=c; } }
}

// ---------- Parser ----------
static void parsePreTxtToGrid(const String &preHtml){
  clearGrid(grid);
  uint16_t fg=C_W, bg=C_K; std::vector<uint16_t> fgs,bgs; int row=0,col=0;
  for(size_t i=0;i<preHtml.length() && row<ROWS;){
    if(preHtml[i]=='<'){
      int close=preHtml.indexOf((char)62,(int)i+1); if(close<0) break;
      String tag=preHtml.substring((int)i+1,close); tag.trim(); bool closing=tag.length()&&tag[0]=='/';
      if(closing){
        String tn=tag.substring(1); tn.trim(); tn.toLowerCase();
        if(tn.startsWith("span")||tn.startsWith("b")){ if(!fgs.empty()){fg=fgs.back(); fgs.pop_back();} if(!bgs.empty()){bg=bgs.back(); bgs.pop_back();} }
      }else{
        String tn=tag; int sp=tn.indexOf(" "); if(sp>0) tn=tn.substring(0,sp); tn.toLowerCase();
        if(tn=="span"||tn=="b"){ fgs.push_back(fg); bgs.push_back(bg); String cl; if(getAttr(tag,"class",cl)) applyClasses(cl,fg,bg); String st; if(getAttr(tag,"style",st)) applyStyleColors(st,fg,bg); }
      }
      i=(size_t)close+1; continue;
    }
    uint32_t cp=nextCodepoint(preHtml,i);
    if(cp==10u){ while(col<COLS) grid[row][col++]=Cell{(char)32,fg,bg,32u}; row++; col=0; continue; }
    if(cp==160u) cp=32u;
    if(cp>=0xE000u && cp<=0xF8FFu){ if(col<COLS) grid[row][col++]=Cell{(char)32,fg,bg,cp}; continue; }
    char ch=iso6937_ascii(cp); if(col<COLS) grid[row][col++]=Cell{ch,fg,bg,cp};
  }
  for(;row<ROWS;++row){ for(col=0;col<COLS;++col) grid[row][col]=Cell{(char)32,C_W,C_K,32u}; }
}

// ---------- <pre class="txt"> ziehen ----------
static bool extractPreByClass(const String &html,const String &cls,String &out){
  String needle=String("<pre class=\"")+cls+"\""; int p=html.indexOf(needle); if(p<0) return false;
  int s=html.indexOf((char)62,p); if(s<0) return false; s++; int e=html.indexOf("</pre>",s); if(e<0) return false; out=html.substring(s,e); return true;
}
static bool extractPre(const String &html,String &pre){
  if(extractPreByClass(html,"txt",pre)) return true;
  int p=html.indexOf("<pre"); if(p>=0){ int q=html.indexOf((char)62,p), e=html.indexOf("</pre>",q+1); if(q>0 && e>q){ pre=html.substring(q+1,e); return true; } }
  return false;
}

// ---------- HTTP/Subpages ----------
static bool httpGet(const String &url,String &body){
  WiFiClientSecure client; client.setInsecure(); HTTPClient http; http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.useHTTP10(true); http.setUserAgent("Mozilla/5.0"); if(!http.begin(client,url)) return false; int code=http.GET();
  Serial.printf("GET %s -> %d\n", url.c_str(), code);
  if(code==200){ body=http.getString(); http.end(); return true; } http.end(); return false;
}
static int detectSubpages(int page,const String &html){ int maxSub=1; for(int s=2;s<=16;s++){ char buf[16]; snprintf(buf,sizeof(buf),"%d_%02d.htm",page,s); if(html.indexOf(buf)>=0) maxSub=s; } return maxSub; }
static String makeUrl(int page,int sub){ char ss[3]; snprintf(ss,sizeof(ss),"%02d",sub); return String("https://www.ndr.de/public/teletext/")+page+"_"+ss+".htm"; }
static bool loadHtml(int page,int sub,String &html){ return httpGet(makeUrl(page,sub),html); }

// ---------- SPIFFS Cache ----------
static String cachePath(int page,int sub){ char p[32]; snprintf(p,sizeof(p),"/tt_%d_%02d.pre",page,sub); return String(p); }
static void   saveCache(int page,int sub,const String &pre){ File f=SPIFFS.open(cachePath(page,sub),FILE_WRITE,true); if(!f) return; f.print(pre); f.close(); }
static bool   loadCache(int page,int sub,String &pre){ File f=SPIFFS.open(cachePath(page,sub),"r"); if(!f) return false; pre=f.readString(); f.close(); return pre.length(); }

// ---------- Grafikrenderer ----------
static void drawNdrThinHLine(int x,int y,uint16_t fg){ int h=(CELL_H>=10)?2:1; int yy=y+(CELL_H-h)/2; FILL_RECT(x,yy,CELL_W,h,fg); }

// 2×3 Mosaik (contiguous)
static void drawMosaic2x3Contiguous(int x,int y,uint32_t mask,uint16_t fg){
  int lw=CELL_W/2, rw=CELL_W-lw; int th=CELL_H/3, mh=CELL_H/3, bh=CELL_H-th-mh;
  if(mask&0x01) FILL_RECT(x,      y,         lw,th,fg);
  if(mask&0x02) FILL_RECT(x+lw,   y,         rw,th,fg);
  if(mask&0x04) FILL_RECT(x,      y+th,      lw,mh,fg);
  if(mask&0x08) FILL_RECT(x+lw,   y+th,      rw,mh,fg);
  if(mask&0x10) FILL_RECT(x,      y+th+mh,   lw,bh,fg);
  if(mask&0x20) FILL_RECT(x+lw,   y+th+mh,   rw,bh,fg);
}

// 2×3 Mosaik (separated)
static void drawMosaic2x3Separated(int x,int y,uint32_t mask,uint16_t fg){
  const int gap = 1;                       // Abstände
  const int aw  = CELL_W - 3*gap;          // verfügbare Breite
  const int ah  = CELL_H - 4*gap;          // verfügbare Höhe
  int lw = aw/2, rw = aw-lw;
  int th = ah/3, mh = ah/3, bh = ah-th-mh;

  int xL = x + gap;
  int xR = x + gap + lw + gap;
  int yT = y + gap;
  int yM = y + gap + th + gap;
  int yB = y + gap + th + gap + mh + gap;

  if(mask&0x01) FILL_RECT(xL, yT, lw, th, fg);
  if(mask&0x02) FILL_RECT(xR, yT, rw, th, fg);
  if(mask&0x04) FILL_RECT(xL, yM, lw, mh, fg);
  if(mask&0x08) FILL_RECT(xR, yM, rw, mh, fg);
  if(mask&0x10) FILL_RECT(xL, yB, lw, bh, fg);
  if(mask&0x20) FILL_RECT(xR, yB, rw, bh, fg);
}

// Unicode Blockgrafik (2580–259F)
static void fillUL(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x,y,hw,hh,c); }
static void fillUR(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x+CELL_W-hw,y,hw,hh,c); }
static void fillLL(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x,y+CELL_H-hh,hw,hh,c); }
static void fillLR(int x,int y,int hw,int hh,uint16_t c){ FILL_RECT(x+CELL_W-hw,y+CELL_H-hh,hw,hh,c); }
static void drawBlockByCodepoint(int x,int y,uint32_t cp,uint16_t fg){
  int hw=(CELL_W+1)/2, hh=(CELL_H+1)/2;
  if(cp>=0x2581 && cp<=0x2588){ int e=(int)(cp-0x2580); int h=(CELL_H*e)/8; FILL_RECT(x,y+(CELL_H-h),CELL_W,h,fg); return; }
  if(cp==0x2594){ int h=(CELL_H+7)/8; FILL_RECT(x,y,CELL_W,h,fg); return; }
  if(cp==0x2580){ FILL_RECT(x,y,CELL_W,CELL_H/2,fg); return; }
  if(cp==0x2588){ FILL_RECT(x,y,CELL_W,CELL_H,fg); return; }
  if(cp>=0x2589 && cp<=0x258F){ int map[7]={7,6,5,4,3,2,1}; int w=(CELL_W*map[(int)(cp-0x2589)])/8; FILL_RECT(x,y,w,CELL_H,fg); return; }
  if(cp==0x2590){ FILL_RECT(x+CELL_W/2,y,CELL_W-CELL_W/2,CELL_H,fg); return; }
  if(cp==0x2595){ int w=(CELL_W+7)/8; FILL_RECT(x+CELL_W-w,y,w,CELL_H,fg); return; }
  switch(cp){
    case 0x2596: fillLL(x,y,hw,hh,fg); return; case 0x2597: fillLR(x,y,hw,hh,fg); return;
    case 0x2598: fillUL(x,y,hw,hh,fg); return; case 0x2599: fillUL(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return;
    case 0x259A: fillUL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return; case 0x259B: fillUL(x,y,hw,hh,fg); fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); return;
    case 0x259C: fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); return; case 0x259D: fillUR(x,y,hw,hh,fg); return;
    case 0x259E: fillUR(x,y,hw,hh,fg); fillLL(x,y,hw,hh,fg); return; case 0x259F: fillUR(x,y,hw,hh,fg); fillLR(x,y,hw,hh,fg); fillUL(x,y,hw,hh,fg); return;
  }
}

// ---------- Diff-Renderer ----------
static void renderDiff(){
  for(int r=0;r<ROWS;r++){
    for(int c=0;c<COLS;c++){
      const Cell &cur=grid[r][c]; Cell &old=prevGrid[r][c];
      if(cur.ch==old.ch && cur.fg==old.fg && cur.bg==old.bg && cur.cp==old.cp) continue;

      int x=c*CELL_W, y=r*CELL_H;
      if(cur.bg!=old.bg || cur.cp!=old.cp){ FILL_RECT(x,y,CELL_W,CELL_H,cur.bg); }

      if(cur.cp>=0xE000u && cur.cp<=0xF8FFu){
        if(cur.cp==0xE00Cu){ drawNdrThinHLine(x,y,cur.fg); }
        else if(cur.cp>=0xE000u && cur.cp<=0xE03Fu){ drawMosaic2x3Contiguous(x,y,cur.cp-0xE000u,cur.fg); }
        else if(cur.cp>=0xE040u && cur.cp<=0xE07Fu){ drawMosaic2x3Separated(x,y,cur.cp-0xE040u,cur.fg); }
        else{ FILL_RECT(x,y,CELL_W,CELL_H,cur.fg); }
      }
      else if((cur.cp>=0x2580 && cur.cp<=0x259F) || cur.cp==0x2588){
        drawBlockByCodepoint(x,y,cur.cp,cur.fg);
      }else{
        int cx=x+((CELL_W-6)/2), cy=y+((CELL_H-8)/2);
        DRAW_CHAR(cx,cy,(uint8_t)cur.ch,cur.fg,cur.bg);
      }
      old=cur;
    }
  }
}

// ---------- Seite laden/anzeigen ----------
static int detectSubpagesOnline(int page){
  String h; if(loadHtml(page,1,h)){ String pre; if(extractPre(h,pre)) saveCache(page,1,pre); return detectSubpages(page,h); }
  return 0;
}
static bool showPage(int page,int sub){
  String html, pre; bool online = loadHtml(page,sub,html);
  if(online){ if(extractPre(html,pre)) saveCache(page,sub,pre); else if(!loadCache(page,sub,pre)) return false; }
  else{ if(!loadCache(page,sub,pre)) return false; }
  parsePreTxtToGrid(pre); renderDiff(); return true;
}
static int getSubCount(int page){
  int n = detectSubpagesOnline(page);
  if(n>0) return n;
  int maxSub=0; for(int s=1;s<=16;s++){ File f=SPIFFS.open(cachePath(page,s),"r"); if(f){ maxSub=s; f.close(); } }
  return maxSub>0?maxSub:1;
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);
  SPIFFS.begin(true);
  ledRGB_init();
  backlightOffIfAvailable();
  INIT_TFT();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WLAN verbinden");
  unsigned long t0=millis(); while(WiFi.status()!=WL_CONNECTED && millis()-t0<10000){ delay(300); Serial.print('.'); }
  Serial.println(); Serial.printf("WLAN: %s\n", WiFi.status()==WL_CONNECTED?"ok":"offline");

  clearGrid(prevGrid);
  FILL_SCREEN(C_K);
}

void loop(){
  for(int i=0;i<NUM_PAGES;i++){
    int page=PAGES[i], subCount=getSubCount(page); if(subCount<1) subCount=1;
    for(int s=1;s<=subCount;s++){
      Serial.printf("Seite %d_%02d\n", page, s);
      if(!showPage(page,s)){
        const char* msg="Offline / kein Cache"; int cx=8, cy=SCREEN_H/2-8; for(int k=0; msg[k]; ++k) DRAW_CHAR(cx+k*8,cy,(uint8_t)msg[k],C_R,C_K);
      }
      for(int t=0;t<20;t++) delay(1000);
    }
  }
}
