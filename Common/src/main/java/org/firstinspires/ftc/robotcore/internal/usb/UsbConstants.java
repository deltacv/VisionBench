/*
Copyright (c) 2018 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcore.internal.usb;

import androidx.annotation.Nullable;

import java.util.Arrays;
import java.util.List;

/**
 * This is an enhancement to android.hardware.usb.UsbConstants.
 */
@SuppressWarnings("WeakerAccess")
public class UsbConstants
    {
    public static final int USB_ENDPOINT_DIR_MASK = 0x80;
    public static final int USB_DIR_OUT = 0;
    public static final int USB_DIR_IN = 0x80;

    public static final int USB_ENDPOINT_NUMBER_MASK = 0x0f;

    public static final int USB_ENDPOINT_XFERTYPE_MASK = 0x03;
    public static final int USB_ENDPOINT_XFER_CONTROL = 0;
    public static final int USB_ENDPOINT_XFER_ISOC = 1;
    public static final int USB_ENDPOINT_XFER_BULK = 2;
    public static final int USB_ENDPOINT_XFER_INT = 3;

    public static final int USB_TYPE_MASK = (0x03 << 5);
    public static final int USB_TYPE_STANDARD = (0x00 << 5);
    public static final int USB_TYPE_CLASS = (0x01 << 5);
    public static final int USB_TYPE_VENDOR = (0x02 << 5);
    public static final int USB_TYPE_RESERVED = (0x03 << 5);

    public static final int USB_CLASS_PER_INTERFACE = 0;
    public static final int USB_CLASS_AUDIO = 1;
    public static final int USB_CLASS_COMM = 2;
    public static final int USB_CLASS_HID = 3;
    public static final int USB_CLASS_PHYSICA = 5;
    public static final int USB_CLASS_STILL_IMAGE = 6;
    public static final int USB_CLASS_PRINTER = 7;
    public static final int USB_CLASS_MASS_STORAGE = 8;
    public static final int USB_CLASS_HUB = 9;
    public static final int USB_CLASS_CDC_DATA = 0x0a;
    public static final int USB_CLASS_CSCID = 0x0b;
    public static final int USB_CLASS_CONTENT_SEC = 0x0d;
    public static final int USB_CLASS_VIDEO = 0x0e;

    public static final int USB_VIDEO_INTERFACE_SUBCLASS_UNDEFINED = 0;
    public static final int USB_VIDEO_INTERFACE_SUBCLASS_CONTROL = 1;
    public static final int USB_VIDEO_INTERFACE_SUBCLASS_STREAMING = 2;
    public static final int USB_VIDEO_INTERFACE_SUBCLASS_INTERFACE_COLLECTION = 3;

    public static final int USB_VIDEO_INTERFACE_PROTOCOL_UNDEFINED = 0;
    public static final int USB_VIDEO_INTERFACE_PROTOCOL_15 = 1;

    public static final int USB_VIDEO_CLASS_DESCRIPTOR_UNDEFINED = 0x20;
    public static final int USB_VIDEO_CLASS_DESCRIPTOR_DEVICE = 0x21;
    public static final int USB_VIDEO_CLASS_DESCRIPTOR_CONFIGURATION = 0x22;
    public static final int USB_VIDEO_CLASS_DESCRIPTOR_STRING = 0x23;
    public static final int USB_VIDEO_CLASS_DESCRIPTOR_INTERFACE = 0x24;
    public static final int USB_VIDEO_CLASS_DESCRIPTOR_ENDPOINT = 0x25;

    public static final int USB_CLASS_WIRELESS_CONTROLLER = 0xe0;
    public static final int USB_CLASS_MISC = 0xef;
    public static final int USB_CLASS_APP_SPEC = 0xfe;
    public static final int USB_CLASS_VENDOR_SPEC = 0xff;

    public static final int USB_INTERFACE_SUBCLASS_BOOT = 1;
    public static final int USB_SUBCLASS_VENDOR_SPEC = 0xff;

    //----------------------------------------------------------------------------------------------

    public static final int VENDOR_ID_MICROSOFT = 0x045E;
    public static final int VENDOR_ID_LOGITECH = 0x046D;
    public static final int VENDOR_ID_ARDUCAM = 0xC45;
    public static final int VENDOR_ID_FTDI = 0x0403;
    public static final int VENDOR_ID_AUSDOM = 3034;

    // https://android.googlesource.com/platform/system/core/+/android-4.4_r1/adb/usb_vendors.c
    // http://www.linux-usb.org/usb.ids
    public static final int VENDOR_ID_GOOGLE = 0x18d1;
    public static final int VENDOR_ID_INTEL = 0x8087;
    public static final int VENDOR_ID_HTC = 0x0bb4;
    public static final int VENDOR_ID_SAMSUNG = 0x04e8;
    public static final int VENDOR_ID_MOTOROLA = 0x22b8;
    public static final int VENDOR_ID_LGE = 0x1004;
    public static final int VENDOR_ID_HUAWEI = 0x12D1;
    public static final int VENDOR_ID_ACER = 0x0502;
    public static final int VENDOR_ID_SONY_ERICSSON = 0x0FCE;
    public static final int VENDOR_ID_FOXCONN = 0x0489;
    public static final int VENDOR_ID_DELL = 0x413c;
    public static final int VENDOR_ID_NVIDIA = 0x0955;
    public static final int VENDOR_ID_GARMIN_ASUS = 0x091E;
    public static final int VENDOR_ID_SHARP = 0x04dd;
    public static final int VENDOR_ID_ZTE = 0x19D2;
    public static final int VENDOR_ID_KYOCERA = 0x0482;
    public static final int VENDOR_ID_PANTECH = 0x10A9;
    public static final int VENDOR_ID_QUALCOMM = 0x05c6;
    public static final int VENDOR_ID_OTGV = 0x2257;
    public static final int VENDOR_ID_NEC = 0x0409;
    public static final int VENDOR_ID_PMC = 0x04DA;
    public static final int VENDOR_ID_TOSHIBA = 0x0930;
    public static final int VENDOR_ID_SK_TELESYS = 0x1F53;
    public static final int VENDOR_ID_KT_TECH = 0x2116;
    public static final int VENDOR_ID_ASUS = 0x0b05;
    public static final int VENDOR_ID_PHILIPS = 0x0471;
    public static final int VENDOR_ID_TI = 0x0451;
    public static final int VENDOR_ID_FUNAI = 0x0F1C;
    public static final int VENDOR_ID_GIGABYTE = 0x0414;
    public static final int VENDOR_ID_IRIVER = 0x2420;
    public static final int VENDOR_ID_COMPAL = 0x1219;
    public static final int VENDOR_ID_T_AND_A = 0x1BBB;
    public static final int VENDOR_ID_LENOVOMOBILE = 0x2006;
    public static final int VENDOR_ID_LENOVO = 0x17EF;
    public static final int VENDOR_ID_VIZIO = 0xE040;
    public static final int VENDOR_ID_K_TOUCH = 0x24E3;
    public static final int VENDOR_ID_PEGATRON = 0x1D4D;
    public static final int VENDOR_ID_ARCHOS = 0x0E79;
    public static final int VENDOR_ID_POSITIVO = 0x1662;
    public static final int VENDOR_ID_FUJITSU = 0x04C5;
    public static final int VENDOR_ID_LUMIGON = 0x25E3;
    public static final int VENDOR_ID_QUANTA = 0x0408;
    public static final int VENDOR_ID_INQ_MOBILE = 0x2314;
    public static final int VENDOR_ID_SONY = 0x054C;
    public static final int VENDOR_ID_LAB126 = 0x1949;
    public static final int VENDOR_ID_YULONG_COOLPAD = 0x1EBF;
    public static final int VENDOR_ID_KOBO = 0x2237;
    public static final int VENDOR_ID_TELEEPOCH = 0x2340;
    public static final int VENDOR_ID_ANYDATA = 0x16D5;
    public static final int VENDOR_ID_HARRIS = 0x19A5;
    public static final int VENDOR_ID_OPPO = 0x22D9;
    public static final int VENDOR_ID_XIAOMI = 0x2717;
    public static final int VENDOR_ID_BYD = 0x19D1;
    public static final int VENDOR_ID_OUYA = 0x2836;
    public static final int VENDOR_ID_HAIER = 0x201E;
    public static final int VENDOR_ID_HISENSE = 0x109b;
    public static final int VENDOR_ID_MTK = 0x0e8d;
    public static final int VENDOR_ID_NOOK = 0x2080;
    public static final int VENDOR_ID_QISDA = 0x1D45;
    public static final int VENDOR_ID_ECS = 0x03fc;
    public static final int VENDOR_ID_GENERIC = 0x1908;

    public static final int PRODUCT_ID_LOGITECH_F310 = 0xC21D;
    public static final int PRODUCT_ID_LOGITECH_C920 = 0x082D;
    public static final int PRODUCT_ID_LOGITECH_C310 = 0x081B;
    public static final int PRODUCT_ID_LOGITECH_C270 = 0x0825;
    public static final int PRODUCT_ID_ARDUCAM_OV9281 = 0x6366;
    public static final int PRODUCT_ID_MICROSOFT_LIFECAM_HD_3000 = 2064;
    public static final int PRODUCT_ID_MICROSOFT_LIFECAM_HD_5000 = 1901;
    public static final int PRODUCT_ID_MICROSOFT_LIFECAM_STUDIO = 1906;
    public static final int PRODUCT_ID_AUSDOM_AW615 = 22704;

    public static final int PRODUCT_ID_MICROSOFT_XBOX360_WIRED = 0x028E;

    public static final int PRODUCT_ID_SONY_DUALSHOCK_4_GEN_1 = 0x05C4;
    public static final int PRODUCT_ID_SONY_DUALSHOCK_4_GEN_2 = 0x09CC;

    /** Some cameras return really meaningless names */
    public static final List<String> manufacturerNamesToIgnore = Arrays.asList("generic");

    public static @Nullable String getManufacturerName(String manufacturer, int vid)
        {
        if (manufacturer == null || manufacturer.isEmpty() || manufacturerNamesToIgnore.contains(manufacturer.toLowerCase()))
            {
            String result = getManufacturerName(vid);
            if (result != null)
                {
                return result;
                }
            }
        return manufacturer;
        }

    public static @Nullable String getManufacturerName(int vid)
        {
        switch (vid)
            {
            case VENDOR_ID_MICROSOFT:   return "Microsoft";
            case VENDOR_ID_LOGITECH:    return "Logitech";
            case VENDOR_ID_FTDI:        return "FTDI";
            case VENDOR_ID_GOOGLE:      return "Google";
            case VENDOR_ID_DELL:        return "Dell";
            case VENDOR_ID_QUALCOMM:    return "Qualcomm";
            case VENDOR_ID_GENERIC:     return "Generic";
            }
        return null;
        }

    protected static int getManufacturerResourceId(int vid)
        {
        return 0;
        }

    public static @Nullable String getProductName(String productName, int vid, int pid)
        {
        if (productName == null || productName.isEmpty())
            {
            String result = getProductName(vid, pid);
            if (result != null)
                {
                return result;
                }
            }
        return productName;
        }

    public static @Nullable String getProductName(int vid, int pid)
        {
        if (vid == VENDOR_ID_LOGITECH)
            {
            switch (pid)
                {
                case PRODUCT_ID_LOGITECH_C920: return "Logitech Webcam C920";
                case PRODUCT_ID_LOGITECH_C310: return "Logitech HD Webcam C310";
                }
            }
        return null;
        }

    protected static int getProductNameResourceId(int vid, int pid)
        {
        return 0;
        }
    }
