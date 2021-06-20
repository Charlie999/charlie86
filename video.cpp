#include <stdio.h>
#include <windows.h>
#include <winspool.h>
#include <thread>
#include <stack>

#define NOREGFUNCS
#include "regs.h"

#include "video.h"
#include "types.h"
#include "common.h"

#include "vga256.h"


int xh, yh;
uint32_t videobuf;

HBITMAP hBitmap;
int frame = 0;

uint16_t modeflags = 0;

HBRUSH blackBrush = (HBRUSH)(GetStockObject(BLACK_BRUSH));

void drawText(HWND hwnd, LPCSTR message, int x, int y) {
    RECT rect;
    HDC wdc = GetWindowDC(hwnd);
    GetClientRect (hwnd, &rect) ;
    HFONT hFont = (HFONT)GetStockObject(ANSI_FIXED_FONT);
    SelectObject(wdc, hFont);
    SetTextColor(wdc,RGB(0xFF, 0xFF, 0xFF));
    SetBkColor(wdc, 0);
    SetBkMode(wdc,OPAQUE);
    rect.left=x;
    rect.top=y;
    DrawText( wdc, message, -1, &rect, DT_NOCLIP  ) ;
    DeleteDC(wdc);
}



int redrawBackground = 0;

#define CHAR_WIDTH 9
#define CHAR_HEIGHT 12

void PaintWindow( HWND hwnd )
{
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint( hwnd, &ps );

    uint8_t* memory = getmemory();

    if (redrawBackground) {
        SelectObject(hdc, blackBrush);
        Rectangle(hdc, 0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
        redrawBackground = 0;
    }

    int statusx = 10;
    int statusy = 300;

    if (modeflags & GRAPHICS_MODE_8BPP) {
        uint8_t *vbuf = (uint8_t *) malloc((xh + 20) * (yh * 20) * 4);
        memset(vbuf, 0x70, (xh + 20) * (yh * 20) * 4);
        for (int y = 0; y < yh; y++) {
            for (int x = 0; x < xh; x++) {
                uint8_t val = memory[((y * xh) + x) + videobuf];
                uint32_t rgb = vga256_to_rgb[val];
                vbuf[4 * (((y + 10) * (xh + 20)) + x + 10) + 2] = (rgb & 0xFF0000) >> 16;
                vbuf[4 * (((y + 10) * (xh + 20)) + x + 10) + 1] = (rgb & 0xFF00) >> 8;
                vbuf[4 * (((y + 10) * (xh + 20)) + x + 10) + 0] = rgb & 0xFF;
                vbuf[4 * (((y + 10) * (xh + 20)) + x + 10) + 3] = 0xFF;
            }
        }

        // draw bg
        HBITMAP map = CreateBitmap(xh + 20, yh + 20, 1, 8 * 4, (void *) vbuf);

        HDC src = CreateCompatibleDC(hdc);
        SelectObject(src, map);

        BitBlt(hdc, 0, 0, xh + 20, yh + 20, src, 0, 0, SRCCOPY);
        DeleteDC(src);

        statusx = 10;
        statusy = yh+55;

        free(vbuf);
    } else if (modeflags & TEXT_MODE_8BPC) {
        for (int y = 0; y < (yh/CHAR_HEIGHT); y++) {
            for (int x = 0; x < (xh/CHAR_WIDTH); x++) {
                if (memory[((y * (xh/CHAR_WIDTH)) + x) + videobuf] < 0x20)
                    memory[((y * (xh/CHAR_WIDTH)) + x) + videobuf] = 0x20;
            }

            char line[(xh/CHAR_WIDTH) + 1];
            line[(xh/CHAR_WIDTH)] = 0;
            snprintf(line, (xh/CHAR_WIDTH), "%s",memory + (y * (xh/CHAR_WIDTH)) + videobuf);
            drawText(hwnd, line, 10, (CHAR_HEIGHT*y)+40);
        }

        statusx = 10;
        statusy = (yh)+55;
    } else if (modeflags & TEXT_MODE_16BPC) {
        for (int y = 0; y < (yh/CHAR_HEIGHT); y++) {
            for (int x = 0; x < (xh/CHAR_WIDTH)*2; x+=2) {
                if (memory[((y * (xh/CHAR_WIDTH)*2) + x) + videobuf] < 0x20)
                    memory[((y * (xh/CHAR_WIDTH)*2) + x) + videobuf] = 0x20;
            }

            char line[(xh/CHAR_WIDTH) + 1];
            line[(xh/CHAR_WIDTH)] = 0;
            //snprintf(line, (xh/CHAR_WIDTH), "%s",memory + (y * (xh/CHAR_WIDTH)) + videobuf);
            for (int c = 0; c < (xh/CHAR_WIDTH)*2; c+=2) {
                line[c/2] = memory[(y * (xh/CHAR_WIDTH)*2) + videobuf + c];
            }
            drawText(hwnd, line, 10, (CHAR_HEIGHT*y)+40);
        }

        statusx = 10;
        statusy = (yh)+55;
    }

    callstack_t callstack = getcallstack();

    callstack_entry_t C0(CALLSTACK_ENTRY_BLANK);
    callstack_entry_t C1(CALLSTACK_ENTRY_BLANK);
    callstack_entry_t C2(CALLSTACK_ENTRY_BLANK);
    callstack_entry_t C3(CALLSTACK_ENTRY_BLANK);

    if (!callstack.empty())
        C0 = callstack.c.at(0);
    if (callstack.size() > 1)
        C1 = callstack.c.at(1);
    if (callstack.size() > 2)
        C2 = callstack.c.at(2);
    if (callstack.size() > 3)
        C3 = callstack.c.at(3);

    char status[1024];
    snprintf(status, 1024, "IP=0x%04X AX=0x%04X BX=0x%04X CX=0x%04X DX=0x%04X BP=0x%04X SP=0x%04X     \nSI=0x%04X DI=0x%04X CS=0x%04X SS=0x%04X DS=0x%04X ES=0x%04X FLAGS=0x%04X    \n%s - CYCLES: %llu   \nL0: %s                                                             \nVIDEO FLAGS: %02X\nSTACK [%08X] %02X %02X %02X %02X %02X %02X %02X %02X\nL1: %s                                                             \nL2: %s                                                             \nL3: %s                                                             \nC0: %04X:%04X => %04X:%04X\nC1: %04X:%04X => %04X:%04X\nC2: %04X:%04X => %04X:%04X\nC3: %04X:%04X => %04X:%04X\nCS: %d                                          \n                                                             ", getip(),
             getregval(REGISTER_AX), getregval(REGISTER_BX), getregval(REGISTER_CX),
             getregval(REGISTER_DX), getregval(REGISTER_BP), getregval(REGISTER_SP),
             getregval(REGISTER_SI), getregval(REGISTER_DI), getregval(REGISTER_CS),
             getregval(REGISTER_SS), getregval(REGISTER_DS), getregval(REGISTER_ES), getflags(),
             (getrunning()!=0)?"RUNNING":"HALTED", getcycles(), getlastinsn(), modeflags,
             getstackptr(), getmemory()[getstackptr()], getmemory()[getstackptr()+1],getmemory()[getstackptr()+2],getmemory()[getstackptr()+3],
             getmemory()[getstackptr()+4],getmemory()[getstackptr()+5],getmemory()[getstackptr()+6],getmemory()[getstackptr()+7],
             getlastinsn2(), getlastinsn3(), getlastinsn4(),
             C0.second.first, C0.second.second, C0.first.first, C0.first.second, C1.second.first, C1.second.second, C1.first.first, C1.first.second,
             C2.second.first, C2.second.second, C2.first.first, C2.first.second, C3.second.first, C3.second.second, C3.first.first, C3.first.second,
             (int)callstack.size());
    drawText(hwnd, status, statusx, statusy);

    SetPixel(hdc, 0, 0, RGB(255 * frame, 255 * frame, 255 * frame));
    frame = frame ? 0 : 1;

    DeleteObject(hdc);
    EndPaint( hwnd, &ps );
}

int latestKey = -1;
int latestKeyParity = 0;

void waitForKey() {
    int firstParity = latestKeyParity;
    while (latestKeyParity == firstParity)
        std::this_thread::yield();
}

int getLatestKey() {
    return latestKey;
}

int isKeyPressed() {
    return latestKey>-1;
}

LRESULT CALLBACK WndProc( HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam )
{
    switch (msg)
    {
        case WM_CREATE:
            SetTimer(hwnd, 1, 20, NULL);
            return 0;
        case WM_KEYDOWN:
            latestKey = wParam;
            latestKeyParity = latestKeyParity?0:1;
            return 0;
        case WM_KEYUP:
            latestKey = -1;
            return 0;
        case WM_CLOSE:
            DestroyWindow( hwnd );
            exit(0);
            return 0;
        case WM_DESTROY:
            PostQuitMessage( 0 );
            return 0;
        case WM_PAINT:
            PaintWindow( hwnd );
            return 0;
        case WM_TIMER:
            InvalidateRect(hwnd, NULL, FALSE);
            return 0;
        case WM_MSG_KILLWITHOUTEXIT:
            DestroyWindow( hwnd );
            hwnd = NULL;
            return 0;
    }
    return DefWindowProc( hwnd, msg, wParam, lParam );
}

HWND hwnd = NULL;
int windowRunning = 0;

DWORD WINAPI WindowThread(LPVOID lpParameter) {
    WNDCLASSA wc =
            {
                    0, WndProc, 0, 0, nullptr,
                    LoadIcon( NULL, IDI_APPLICATION ),
                    LoadCursor( NULL, IDC_ARROW ),
                    static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)), // background color == black
                    nullptr, // no menu
                    "Charlie86WindowClass"
            };

    ATOM wClass = RegisterClassA( &wc );
    if (!wClass)
    {
        fprintf( stderr, "%s\n", "Couldn’t create Window Class" );
        exit(0);
    }

    hwnd = CreateWindowA(
            MAKEINTATOM( wClass ),
            "CHARLIE86 8086 emulator",
            WS_OVERLAPPEDWINDOW,
            xh, yh, 800, 600,
            NULL,
            NULL,
            GetModuleHandle( NULL ),  // EXE's HINSTANCE
            NULL
    );
    if (!hwnd)
    {
        fprintf( stderr, "%ld\n", GetLastError() );
        fprintf( stderr, "%s\n", "Failed to create Window" );
        exit(0);
    }

    ShowWindow( hwnd, SW_SHOWNORMAL );

    MSG msg;
    while (GetMessage( &msg, NULL, 0, 0 ) > 0 && windowRunning)
    {
        TranslateMessage( &msg );
        DispatchMessage( &msg );
    }
    DestroyWindow(hwnd);

    return 1;
}

HANDLE windowThreadHandle;

void initvideo(int xres, int yres, uint32_t startaddr, uint16_t flags){
    xh = xres;
    yh = yres;
    videobuf = startaddr;
    modeflags = flags;

    if (flags&TEXT_MODE_8BPC || flags&TEXT_MODE_16BPC) {
        xh*=CHAR_WIDTH;
        yh*=CHAR_HEIGHT;
    }

    printf("VM video initialized, xh=%d, yh=%d, vbuf=0x%X, vsz=%d\n",xh,yh,videobuf,xh*yh);

    if (!hwnd) {
        windowRunning = 1;

        windowThreadHandle = CreateThread(
                NULL,
                0,
                WindowThread,
                NULL,
                0,
                NULL);
        if (windowThreadHandle == NULL) {
            printf("error creating window thread");
            DestroyWindow(hwnd);
            exit(0);
        }
    } else {
        redrawBackground = 1;
    }
}

void killvideo() {
    windowRunning = 0;

    SendMessage(hwnd, WM_MSG_KILLWITHOUTEXIT, 0, 0);
}