"""Unit tests for DroneKit Vehicle class."""

import pytest
from unittest.mock import Mock, MagicMock, patch
from dronekit.vehicle import Vehicle, Location, GPSInfo, Battery, VehicleMode
from dronekit.exceptions import VehicleNotConnectedError, ArmingError


class TestLocation:
    """Tests for Location class."""
    
    def test_location_init(self):
        """Test Location initialization."""
        loc = Location(lat=47.6, lon=-122.3, alt=100.0)
        assert loc.lat == 47.6
        assert loc.lon == -122.3
        assert loc.alt == 100.0
    
    def test_location_repr(self):
        """Test Location string representation."""
        loc = Location(lat=47.6, lon=-122.3, alt=100.0)
        assert "47.6" in repr(loc)
        assert "-122.3" in repr(loc)
        assert "100.0" in repr(loc)


class TestGPSInfo:
    """Tests for GPSInfo class."""
    
    def test_gps_info_init(self):
        """Test GPSInfo initialization."""
        gps = GPSInfo()
        assert gps.fix_type == 0
        assert gps.num_sat == 0
    
    def test_gps_info_repr(self):
        """Test GPSInfo string representation."""
        gps = GPSInfo()
        gps.fix_type = 3
        gps.num_sat = 10
        assert "3" in repr(gps)
        assert "10" in repr(gps)


class TestBattery:
    """Tests for Battery class."""
    
    def test_battery_init(self):
        """Test Battery initialization."""
        battery = Battery()
        assert battery.voltage == 0.0
        assert battery.current == 0.0
        assert battery.level == 0
    
    def test_battery_repr(self):
        """Test Battery string representation."""
        battery = Battery()
        battery.voltage = 12.6
        battery.level = 85
        assert "12.6" in repr(battery)
        assert "85" in repr(battery)


class TestVehicleMode:
    """Tests for VehicleMode class."""
    
    def test_mode_init(self):
        """Test VehicleMode initialization."""
        mode = VehicleMode("GUIDED")
        assert mode.name == "GUIDED"
    
    def test_mode_equality_string(self):
        """Test VehicleMode equality with string."""
        mode = VehicleMode("GUIDED")
        assert mode == "GUIDED"
        assert mode != "AUTO"
    
    def test_mode_equality_mode(self):
        """Test VehicleMode equality with another VehicleMode."""
        mode1 = VehicleMode("GUIDED")
        mode2 = VehicleMode("GUIDED")
        mode3 = VehicleMode("AUTO")
        assert mode1 == mode2
        assert mode1 != mode3


class TestVehicle:
    """Tests for Vehicle class."""
    
    def test_vehicle_init(self):
        """Test Vehicle initialization."""
        vehicle = Vehicle("127.0.0.1:14550")
        assert vehicle._connection_string == "127.0.0.1:14550"
        assert not vehicle._connected
        assert not vehicle._armed
    
    def test_vehicle_repr(self):
        """Test Vehicle string representation."""
        vehicle = Vehicle("127.0.0.1:14550")
        repr_str = repr(vehicle)
        assert "127.0.0.1:14550" in repr_str
        assert "disconnected" in repr_str
    
    def test_check_connected_raises(self):
        """Test that operations on disconnected vehicle raise error."""
        vehicle = Vehicle("127.0.0.1:14550")
        
        with pytest.raises(VehicleNotConnectedError):
            _ = vehicle.armed
        
        with pytest.raises(VehicleNotConnectedError):
            _ = vehicle.location
        
        with pytest.raises(VehicleNotConnectedError):
            vehicle.simple_takeoff(10)
    
    def test_context_manager(self):
        """Test Vehicle as context manager."""
        vehicle = Vehicle("127.0.0.1:14550")
        
        with patch.object(vehicle, 'close') as mock_close:
            with vehicle:
                pass
            mock_close.assert_called_once()
    
    def test_add_attribute_listener(self):
        """Test adding attribute listeners."""
        vehicle = Vehicle("127.0.0.1:14550")
        callback = Mock()
        
        vehicle.add_attribute_listener('armed', callback)
        assert 'armed' in vehicle._attribute_listeners
        assert callback in vehicle._attribute_listeners['armed']
    
    def test_remove_attribute_listener(self):
        """Test removing attribute listeners."""
        vehicle = Vehicle("127.0.0.1:14550")
        callback = Mock()
        
        vehicle.add_attribute_listener('armed', callback)
        vehicle.remove_attribute_listener('armed', callback)
        assert callback not in vehicle._attribute_listeners.get('armed', [])
    
    def test_location_property(self):
        """Test location property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._location = Location(47.6, -122.3, 100.0)
        
        loc = vehicle.location
        assert loc.lat == 47.6
        assert loc.lon == -122.3
        assert loc.alt == 100.0
    
    def test_gps_property(self):
        """Test GPS property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._gps.fix_type = 3
        vehicle._gps.num_sat = 12
        
        gps = vehicle.gps_0
        assert gps.fix_type == 3
        assert gps.num_sat == 12
    
    def test_battery_property(self):
        """Test battery property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._battery.voltage = 12.6
        vehicle._battery.level = 85
        
        battery = vehicle.battery
        assert battery.voltage == 12.6
        assert battery.level == 85
    
    def test_mode_property(self):
        """Test mode property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._mode = VehicleMode("GUIDED")
        
        mode = vehicle.mode
        assert mode.name == "GUIDED"
    
    def test_armed_property(self):
        """Test armed property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._armed = True
        
        assert vehicle.armed is True
    
    def test_is_armable_property(self):
        """Test is_armable property."""
        vehicle = Vehicle("127.0.0.1:14550")
        vehicle._connected = True
        vehicle._is_armable = True
        
        assert vehicle.is_armable is True
