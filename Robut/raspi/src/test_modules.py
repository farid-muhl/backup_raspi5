"""
Test Modules - Individual Component Testing
Test setiap modul secara terpisah sebelum full integration

Usage:
    python3 test_modules.py
"""

import sys
import time

def test_vision():
    """Test vision system"""
    print("\n" + "="*60)
    print("   TESTING VISION SYSTEM")
    print("="*60)
    
    try:
        from vision import VisionSystem
        
        vision = VisionSystem()
        
        print("\n[TEST] Capturing frame...")
        frame = vision.capture_frame()
        
        if frame is None:
            print("[FAIL] Failed to capture frame")
            return False
        
        print(f"[PASS] Frame captured: {frame.shape}")
        
        print("\n[TEST] Detecting all objects...")
        bottles = vision.detect_bottles(frame)
        env = vision.detect_environment(frame)
        
        print(f"[INFO] Detected {len(bottles)} bottles")
        print(f"[INFO] Floor color: {env['floor']}")
        print(f"[INFO] Detected {len(env['obstacles'])} obstacles and {len(env['pots'])} pots")
        
        print("\n[TEST] Visualization...")
        vision.draw_debug(frame, bottles, env)
        
        print("\n[PASS] Vision system test successful!")
        print("Press any key in image window to continue...")
        
        import cv2
        cv2.waitKey(0)
        
        vision.close()
        return True
    
    except Exception as e:
        print(f"[FAIL] Vision test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_lidar():
    from lidar_nav import LidarNavigator
    import time
    import traceback

    print("\n[TEST] Menguji Modul Lidar Navigator...")
    lidar = LidarNavigator(port="/dev/ttyUSB0", baudrate=115200)
    
    try:
        if lidar.start():
            print("[TEST] Mengambil data pemindaian selama 5 detik...")
            try:
                for _ in range(10):
                    time.sleep(1)
                    sectors = lidar.get_sectors()
                    print(f"Jarak Depan: {sectors['front']:.1f} cm | Rintangan Terdekat: {lidar.closest_obstacle:.1f} cm")
            except KeyboardInterrupt:
                print("\n[TEST] Dihentikan oleh user.")
            finally:
                lidar.stop()
                print("[TEST] Pengujian Lidar selesai.")
            
            return True
        else:
            print("[ERROR] Lidar gagal dimulai saat pengujian.")
            
            return False
            
    except Exception as e:
        print(f"[FAIL] LiDAR test failed: {e}")
        traceback.print_exc()
        return False

def test_serial():
    from serial_comm import ESPController
    import time
    import traceback

    print("\n[TEST] Menguji Komunikasi Serial ESP32...")
    try:
        # PENTING: Sesuaikan port dengan posisi ESP32 Anda. 
        # Jika Lidar di /dev/ttyUSB0, ESP32 mungkin di /dev/ttyUSB1 atau /dev/ttyAMA0
        esp = ESPController(port='/dev/ttyUSB1')
        
        # Beri waktu agar koneksi serial stabil
        time.sleep(2)
        
        print("[TEST] Mengirim perintah maju...")
        esp.forward(50)
        time.sleep(1)
        
        print("[TEST] Mengirim perintah geser kiri (slide_left)...")
        if hasattr(esp, 'slide_left'):
            esp.slide_left(50)
        else:
            print("[WARNING] Fungsi slide_left belum ada di ESPController.")
        time.sleep(1)
        
        print("[TEST] Menghentikan motor...")
        esp.stop()
        
        # Pastikan ada metode close() di ESPController Anda
        if hasattr(esp, 'close'):
            esp.close()
            
        print("[TEST] Pengujian Komunikasi Serial selesai.")
        return True
        
    except Exception as e:
        print(f"[FAIL] Pengujian Serial gagal: {e}")
        traceback.print_exc()
        return False
def test_utils():
    """Test utility functions"""
    print("\n" + "="*60)
    print("   TESTING UTILITY FUNCTIONS")
    print("="*60)
    
    try:
        from utils import PIDController, Timer, Logger, StuckDetector
        
        print("\n[TEST] PID Controller...")
        pid = PIDController(kp=0.5, ki=0.0, kd=0.1)
        
        for i in range(5):
            error = 50 - i * 10
            output = pid.update(error, dt=0.1)
            print(f"  Error: {error:5.1f} -> Output: {output:6.2f}")
        
        print("[PASS] PID Controller working")
        
        print("\n[TEST] Timer...")
        timer = Timer(1.0)
        print(f"  Started timer (1.0s timeout)")
        time.sleep(0.5)
        print(f"  Elapsed: {timer.elapsed():.2f}s, Timeout: {timer.is_timeout()}")
        time.sleep(0.6)
        print(f"  Elapsed: {timer.elapsed():.2f}s, Timeout: {timer.is_timeout()}")
        print("[PASS] Timer working")
        
        print("\n[TEST] Logger...")
        logger = Logger("TestLog", log_to_file=False)
        logger.debug("Debug message")
        logger.info("Info message")
        logger.warning("Warning message")
        logger.error("Error message")
        print("[PASS] Logger working")
        
        print("\n[TEST] StuckDetector...")
        stuck = StuckDetector(window_size=5, threshold=10)
        
        # Moving
        for i in range(5):
            is_stuck = stuck.update((i*10, i*10))
            print(f"  Position ({i*10}, {i*10}): Stuck={is_stuck}")
        
        # Stuck
        for i in range(5):
            is_stuck = stuck.update((50, 50))
            print(f"  Position (50, 50): Stuck={is_stuck}")
        
        print("[PASS] StuckDetector working")
        
        print("\n[PASS] All utility tests successful!")
        return True
    
    except Exception as e:
        print(f"[FAIL] Utils test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_config():
    """Test configuration loading"""
    print("\n" + "="*60)
    print("   TESTING CONFIGURATION")
    print("="*60)
    
    try:
        import config
        
        print("\n[INFO] Configuration values:")
        print(f"  Serial Port: {config.SERIAL_PORT}")
        print(f"  Serial Baud: {config.SERIAL_BAUD}")
        print(f"  LiDAR Port: {config.LIDAR_PORT}")
        print(f"  Camera: {config.FRAME_WIDTH}x{config.FRAME_HEIGHT}")
        print(f"  Max Speed: {config.MAX_SPEED}")
        print(f"  Max Bottles: {config.MAX_BOTTLES}")
        
        print(f"\n[INFO] Color Ranges:")
        for key in list(config.COLOR_RANGES.keys())[:5]:
            print(f"  {key}: {config.COLOR_RANGES[key]}")
        print(f"  ... and {len(config.COLOR_RANGES)-5} more")
        
        print(f"\n[INFO] Calibration Status:")
        print(f"  Color Calibrated: {config.COLOR_CALIBRATED}")
        print(f"  Distance Calibrated: {config.DISTANCE_CALIBRATED}")
        
        if not config.COLOR_CALIBRATED:
            print("\n[WARNING] Colors not calibrated! Run tools/color_calibration.py")
        
        if not config.DISTANCE_CALIBRATED:
            print("\n[WARNING] Distance not calibrated! Run tools/distance_calibration.py")
        
        print("\n[PASS] Configuration loaded successfully!")
        return True
    
    except Exception as e:
        print(f"[FAIL] Config test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*60)
    print("   AUTONOMOUS ROBOT - MODULE TESTING")
    print("="*60)
    print("\nThis will test each module individually")
    print("Make sure hardware is connected before starting\n")
    
    results = {}
    
    # Test config first
    print("\n1. Configuration Test")
    results['config'] = test_config()
    
    if not results['config']:
        print("\n[ERROR] Configuration test failed. Fix config.py first!")
        return
    
    # Test utils (no hardware needed)
    print("\n2. Utilities Test")
    results['utils'] = test_utils()
    
    # Ask user which hardware tests to run
    print("\n" + "="*60)
    print("Hardware Tests")
    print("="*60)
    
    print("\n3. Vision System Test")
    print("   Requires: Raspberry Pi Camera")
    test_vision_input = input("   Run vision test? (y/n): ")
    if test_vision_input.lower().startswith('y'):
        results['vision'] = test_vision()
    
    print("\n4. LiDAR System Test")
    print("   Requires: YDLIDAR YDS2MD-1 (or use simulator)")
    test_lidar_input = input("   Run LiDAR test? (y/n): ")
    if test_lidar_input.lower().startswith('y'):
        results['lidar'] = test_lidar()
    
    print("\n5. ESP32 Serial Communication Test")
    print("   Requires: ESP32 connected via USB")
    test_serial_input = input("   Run serial test? (y/n): ")
    if test_serial_input.lower().startswith('y'):
        results['serial'] = test_serial()
    
    # Summary
    print("\n" + "="*60)
    print("   TEST SUMMARY")
    print("="*60)
    
    for module, passed in results.items():
        status = "PASS" if passed else "FAIL"
        symbol = "✓" if passed else "✗"
        print(f"  {symbol} {module:15s}: {status}")
    
    total = len(results)
    # Mengonversi nilai None menjadi False (0) agar aman dijumlahkan
    passed = sum(bool(v) for v in results.values())
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n[SUCCESS] All tests passed! System ready for autonomous mission.")
    else:
        print("\n[WARNING] Some tests failed. Fix issues before running full mission.")


if __name__ == "__main__":
    try:
        run_all_tests()
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Testing interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
