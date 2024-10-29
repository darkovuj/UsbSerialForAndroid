using Android.Hardware.Usb;
using Android.Health.Connect.DataTypes.Units;
using Android.Nfc;
using Android.OS;
using Android.Util;
using Hoho.Android.UsbSerial.Driver;
using Java.Lang;
using Java.Util;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using static Android.Telephony.CarrierConfigManager;

namespace Hoho.Android.UsbSerial.driver
{

    public class GsmModemSerialDriver : UsbSerialDriver
    {


        private UsbDevice mDevice;
        private UsbSerialPort mPort;


        public GsmModemSerialDriver(UsbDevice mDevice)
        {
            this.mDevice = mDevice;
            mPort = new GsmModemSerialPort(mDevice, 0);
        }

        public class GsmModemSerialPort : CommonUsbSerialPort
        {
            private string TAG => (Driver as GsmModemSerialPort)?.TAG;
            private const int USB_WRITE_TIMEOUT_MILLIS = 5000;
            private static int MAX_READ_SIZE = 16 * 1024; // = old bulkTransfer limit

            private UsbInterface mDataInterface;
            private UsbEndpoint mReadEndpoint;
            private UsbEndpoint mWriteEndpoint;

            public GsmModemSerialPort(UsbDevice device, int portNumber) : base(device, portNumber)
            {
            }

            public override void Open(UsbDeviceConnection connection)
            {

                if (mConnection != null)
                {
                    throw new IOException("Already opened.");
                }

                mConnection = connection;
                Boolean opened = false;
                try
                {
                    for (int i = 0; i < mDevice.InterfaceCount; i++)
                    {
                        UsbInterface usbIface = mDevice.GetInterface(i);
                        if (mConnection.ClaimInterface(usbIface, true))
                        {
                            Log.Debug(TAG, $"claimInterface {i} SUCCESS");
                        }
                        else
                        {
                            Log.Debug(TAG, $"claimInterface {i} FAIL");
                        }
                    }

                    UsbInterface dataIface = mDevice.GetInterface(mDevice.InterfaceCount - 1);
                    for (int i = 0; i < dataIface.EndpointCount; i++)
                    {
                        UsbEndpoint ep = dataIface.GetEndpoint(i);
                        if (ep.Type == (UsbAddressing)UsbSupport.UsbEndpointXferBulk)
                        {
                            if (ep.Direction == (UsbAddressing)UsbSupport.UsbDirIn)
                            {
                                mReadEndpoint = ep;
                            }
                            else
                            {
                                mWriteEndpoint = ep;
                            }
                        }

                        initGsmModem();
                    }
                }
                finally
                {
                    if (!opened)
                    {
                        try
                        {
                            Close();
                        }
                        catch (IOException e)
                        {
                            // Ignore IOExceptions during close()
                        }
                    }
                }
            }


            public override void Close()
            {
                if (mConnection == null)
                {
                    throw new IOException("Already closed");
                }

                // TODO: nothing sended on close, maybe needed?

                try
                {
                    mConnection.Close();
                }
                finally
                {
                    mConnection = null;
                }
            }

            protected void testConnection()
            {
               
                byte[] buf = new byte[2];
                int len = mConnection.ControlTransfer((UsbAddressing) 0x80 /*DEVICE*/, 0 /*GET_STATUS*/, 0, 0, buf, buf.length, 200);
                if (len < 0)
                    throw new IOException(msg);
            }

            private int initGsmModem()
            {

                int len = mConnection.ControlTransfer((UsbAddressing)0x21, 0x22, 0x01, 0, null, 0, 5000);
                if (len < 0)
                {
                    throw new IOException("init failed");
                }
                return len;
            }

            public override int Read(byte[] dest, int timeoutMillis)
            {
                testConnection(false);
                if (length <= 0)
                {
                    throw new IllegalArgumentException("Read length too small");
                }
                length = Math.min(length, dest.length);
                final int nread;
                if (timeout != 0)
                {
                    // bulkTransfer will cause data loss with short timeout + high baud rates + continuous transfer
                    //   https://stackoverflow.com/questions/9108548/android-usb-host-bulktransfer-is-losing-data
                    // but mConnection.requestWait(timeout) available since Android 8.0 es even worse,
                    // as it crashes with short timeout, e.g.
                    //   A/libc: Fatal signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr 0x276a in tid 29846 (pool-2-thread-1), pid 29618 (.usbserial.test)
                    //     /system/lib64/libusbhost.so (usb_request_wait+192)
                    //     /system/lib64/libandroid_runtime.so (android_hardware_UsbDeviceConnection_request_wait(_JNIEnv*, _jobject*, long)+84)
                    // data loss / crashes were observed with timeout up to 200 msec
                    int readMax = Math.Min(length, MAX_READ_SIZE);
                    nread = mConnection.BulkTransfer(mReadEndpoint, dest, readMax, USB_WRITE_TIMEOUT_MILLIS);
                    // Android error propagation is improvable:
                    //  nread == -1 can be: timeout, connection lost, buffer to small, ???
                    if (nread == -1 )
                        testConnection( );

                }
                else
                {
                    final ByteBuffer buf = ByteBuffer.wrap(dest, 0, length);
                    if (!mUsbRequest.queue(buf, length))
                    {
                        throw new IOException("Queueing USB request failed");
                    }
                    final UsbRequest response = mConnection.requestWait();
                    if (response == null)
                    {
                        throw new IOException("Waiting for USB request failed");
                    }
                    nread = buf.position();
                    // Android error propagation is improvable:
                    //   response != null & nread == 0 can be: connection lost, buffer to small, ???
                    if (nread == 0)
                    {
                        testConnection(true);
                    }
                }
                return Math.max(nread, 0);
            }

            public override int Write(byte[] src, int timeoutMillis)
            {
                int offset = 0;
                long startTime = MonotonicClock.millis();
                length = Math.min(length, src.length);

                testConnection(false);
                while (offset < length)
                {
                    int requestTimeout;
                    final int requestLength;
                    final int actualLength;

                    synchronized(mWriteBufferLock) {
                        final byte[] writeBuffer;

                        if (mWriteBuffer == null)
                        {
                            mWriteBuffer = new byte[mWriteEndpoint.getMaxPacketSize()];
                        }
                        requestLength = Math.min(length - offset, mWriteBuffer.length);
                        if (offset == 0)
                        {
                            writeBuffer = src;
                        }
                        else
                        {
                            // bulkTransfer does not support offsets, make a copy.
                            System.arraycopy(src, offset, mWriteBuffer, 0, requestLength);
                            writeBuffer = mWriteBuffer;
                        }
                        if (timeout == 0 || offset == 0)
                        {
                            requestTimeout = timeout;
                        }
                        else
                        {
                            requestTimeout = (int)(startTime + timeout - MonotonicClock.millis());
                            if (requestTimeout == 0)
                                requestTimeout = -1;
                        }
                        if (requestTimeout < 0)
                        {
                            actualLength = -2;
                        }
                        else
                        {
                            actualLength = mConnection.bulkTransfer(mWriteEndpoint, writeBuffer, requestLength, requestTimeout);
                        }
                    }
                    long elapsed = MonotonicClock.millis() - startTime;
                    if (DEBUG)
                    {
                        Log.d(TAG, "Wrote " + actualLength + "/" + requestLength + " offset " + offset + "/" + length + " time " + elapsed + "/" + requestTimeout);
                    }
                    if (actualLength <= 0)
                    {
                        String msg = "Error writing " + requestLength + " bytes at offset " + offset + " of total " + src.length + " after " + elapsed + "msec, rc=" + actualLength;
                        if (timeout != 0)
                        {
                            // could be buffer full because: writing to fast, stopped by flow control
                            testConnection(elapsed < timeout, msg);
                            throw new SerialTimeoutException(msg, offset);
                        }
                        else
                        {
                            throw new IOException(msg);

                        }
                    }
                    offset += actualLength;
                }
            }

            public override void SetParameters(int baudRate, int dataBits, StopBits stopBits, Parity parity)
            {
                throw new NotImplementedException();
            }

            public override bool GetCD()
            {
                throw new NotImplementedException();
            }

            public override bool GetCTS()
            {
                throw new NotImplementedException();
            }

            public override bool GetDSR()
            {
                throw new NotImplementedException();
            }

            public override bool GetDTR()
            {
                throw new NotImplementedException();
            }

            public override void SetDTR(bool value)
            {
                throw new NotImplementedException();
            }

            public override bool GetRI()
            {
                throw new NotImplementedException();
            }

            public override bool GetRTS()
            {
                throw new NotImplementedException();
            }

            public override void SetRTS(bool value)
            {
                throw new NotImplementedException();
            }

            public override IUsbSerialDriver GetDriver()
            {
                throw new NotImplementedException();
            }
        }
    }

}
