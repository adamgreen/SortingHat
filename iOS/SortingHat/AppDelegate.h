/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 */

#import <UIKit/UIKit.h>
#import <CoreBluetooth/CoreBluetooth.h>

@interface AppDelegate : UIResponder <UIApplicationDelegate, CBCentralManagerDelegate, CBPeripheralDelegate>
{
    CBCentralManager *manager;
    CBPeripheral *peripheral;
    CBCharacteristic* sendDataWriteCharacteristic;
    BOOL isBlePowerOn;
    BOOL scanOnBlePowerOn;
}

@property (strong, nonatomic) UIWindow *window;

- (void) startScan;
- (void) stopScan;
- (void) clearPeripheral;
- (void) updateStatus:(NSString*)string;
- (void) sendToSortingHat:(const char*)pData;

@end

