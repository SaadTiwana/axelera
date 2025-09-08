#!/usr/bin/env python3
"""
Test script to verify configuration changes
"""
from config import ConfigManager

def test_config():
    print("Testing configuration loading...")

    try:
        c = ConfigManager()

        print(f"Scan cycles: {c.scan_area.scan_cycles}")
        print(f"Min overlap: {c.scan_area.min_overlap_percentage}")
        print(f"Restricted areas: {len(c.restricted_areas)}")

        for area in c.restricted_areas:
            print(f"  {area.name}: {area.classes}")

        print("✅ Configuration test passed!")

    except Exception as e:
        print(f"❌ Configuration test failed: {e}")
        return False

    return True

if __name__ == "__main__":
    test_config()