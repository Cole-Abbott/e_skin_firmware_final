import sys
import numpy as np
import csv
import usb.core
import usb.util
import usb.backend.libusb1

ADC_SAMPLES = 766
ADC_PERIOD = 0.16  # [us]

def init_usb_device():
    path_to_libusb = '/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib'
    backend = usb.backend.libusb1.get_backend(find_library=lambda x: path_to_libusb)
    dev = usb.core.find(idVendor=0x04d8, idProduct=0x0053, backend=backend)
    if dev is None:
        raise ValueError('Device not found')
    dev.set_configuration()
    return dev

def read_usb_data(dev):
    try:
        data = dev.read(0x81, 1536, timeout=1000)
    except usb.core.USBError as e:
        if e.errno == 60:
            raise ValueError("Operation timed out")
        elif e.errno == 19:
            raise ValueError("Device not found")
        else:
            raise
    adc_data = np.zeros(ADC_SAMPLES)
    for i in range(ADC_SAMPLES):
        adc_data[i] = data[i * 2 + 2] + (data[i * 2 + 3] << 8)
    return adc_data

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} output.csv num_iterations")
        sys.exit(1)
    output_file = sys.argv[1]
    num_iterations = int(sys.argv[2])

    dev = init_usb_device()

    # Collect all iterations first
    all_data = []
    for i in range(num_iterations):
        try:
            adc_data = read_usb_data(dev)
            all_data.append(adc_data.astype(int))
            print(f"Saved iteration {i+1}/{num_iterations}")
        except Exception as e:
            print(f"Error on iteration {i+1}: {e}")
            break

    # Transpose so each column is one iteration
    all_data = np.array(all_data).T  # Shape: (ADC_SAMPLES, num_iterations)

    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header: iteration_0, iteration_1, ...
        header = [f"iteration_{i}" for i in range(all_data.shape[1])]
        writer.writerow(header)
        for row in all_data:
            writer.writerow(row)