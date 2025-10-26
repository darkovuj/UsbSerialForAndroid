/* Copyright 2017 Tyler Technologies Inc.
 *
 * Project home page: https://github.com/anotherlab/xamarin-usb-serial-for-android
 * Portions of this library are based on usb-serial-for-android (https://github.com/mik3y/usb-serial-for-android).
 * Portions of this library are based on Xamarin USB Serial for Android (https://bitbucket.org/lusovu/xamarinusbserial).
 */

using System;
using System.Threading;
using Android.Hardware.Usb;
using System.Threading.Tasks;
using Hoho.Android.UsbSerial.Driver;
using Serilog;

namespace Hoho.Android.UsbSerial.Extensions
{
    public class SerialInputOutputManager : IDisposable
    {
        const int READ_WAIT_MILLIS = 200;
        const int DEFAULT_BUFFERSIZE = 4096;
        const int DEFAULT_BAUDRATE = 9600;
        const int DEFAULT_DATABITS = 8;
        const Parity DEFAULT_PARITY = Parity.None;
        const StopBits DEFAULT_STOPBITS = StopBits.One;

        readonly UsbSerialPort port;
        byte[] buffer;
        CancellationTokenSource cancelationTokenSource;
        bool isOpen;

        public SerialInputOutputManager(UsbSerialPort port)
        {
            this.port = port;
            BaudRate = DEFAULT_BAUDRATE;
            Parity = DEFAULT_PARITY;
            DataBits = DEFAULT_DATABITS;
            StopBits = DEFAULT_STOPBITS;
        }

        public int BaudRate { get; set; }

        public Parity Parity { get; set; }

        public int DataBits { get; set; }

        public StopBits StopBits { get; set; }

        public event EventHandler<SerialDataReceivedArgs> DataReceived;

        public event EventHandler<UnhandledExceptionEventArgs> ErrorReceived;

        public event EventHandler Closed;

        public void Open(UsbManager usbManager, int bufferSize = DEFAULT_BUFFERSIZE)
        {
            if (disposed)
                throw new ObjectDisposedException(GetType().Name);
            if (IsOpen)
                throw new InvalidOperationException();

            Log.Verbose("Opening serial port");


            var connection = usbManager.OpenDevice(port.GetDriver().GetDevice());
            if (connection == null)
                throw new Java.IO.IOException("Failed to open device");
            isOpen = true;

            buffer = new byte[bufferSize];
            port.Open(connection);
            port.SetParameters(BaudRate, DataBits, StopBits, Parity);

            cancelationTokenSource = new CancellationTokenSource();
            var cancelationToken = cancelationTokenSource.Token;
            cancelationToken.Register(() => Log.Debug("Cancellation Requested"));

            Task.Run(() =>
            {
                int threadID = System.Threading.Thread.CurrentThread.ManagedThreadId;
                Log.Information("<{0}> USB serial communication task Started!", threadID);
                try
                {
                    while (true)
                    {
                        cancelationToken.ThrowIfCancellationRequested();

                        Step(); // execute step
                    }
                }
                catch (OperationCanceledException)
                {

                }
                catch (Exception e)
                {
                    Log.Warning(e, "<{0}> Task ending due to exception: " + e.Message, threadID);
                    ErrorReceived.Raise(this, new UnhandledExceptionEventArgs(e, false));
                }
                finally
                {
                   // onClose();
                    Log.Debug("<{0}> USB serial communication task  Ended!", threadID);
                }
            }, cancelationToken);
        }

        public void Close()
        {
            if (disposed)
                throw new ObjectDisposedException(GetType().Name);
            if (!IsOpen)
                throw new InvalidOperationException();

            onClose();
            // cancel task
            cancelationTokenSource.Cancel();
        }

        void onClose()
        {
            if (!isOpen)
                return;

            Log.Verbose("Closing serial channel");

            Closed?.Invoke(this, EventArgs.Empty);

            port.Close();
            buffer = null;
            isOpen = false;
        }
        public bool IsOpen => isOpen;

        void Step()
        {
            // handle incoming data.
            var len = port.Read(buffer, READ_WAIT_MILLIS);
            if (len > 0)
            {
                Log.Debug("Read data len=" + len);

                var data = new byte[len];
                Array.Copy(buffer, data, len);
                DataReceived.Raise(this, new SerialDataReceivedArgs(data));
            }
        }


        #region Dispose pattern implementation

        bool disposed;

        protected virtual void Dispose(bool disposing)
        {
            if (disposed)
                return;

            if (disposing && (cancelationTokenSource != null))
            {
                Close();
            }

            disposed = true;
        }

        ~SerialInputOutputManager()
        {
            Dispose(false);
        }

        #region IDisposable implementation

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        #endregion

        #endregion

    }
}