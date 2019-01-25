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

#import "AppDelegate.h"
#import "ViewController.h"

// The Nordic BLE UART service
#define NORDIC_BLE_UART_SERVICE    "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

// Characteristic of NORDIC_BLE_UART_SERVICE to which data is sent to SortingHat.
#define NORDIC_BLE_UART_WRITE_CHARACTERISTIC     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// The name found in the Sorting Hat BLE advertisement.
#define SORTING_HAT_NAME "SortingHat"

@interface AppDelegate ()

@end

@implementation AppDelegate


- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
    manager = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
    [self startScan];

    return YES;
}


- (void)applicationWillResignActive:(UIApplication *)application {
    // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
    // Use this method to pause ongoing tasks, disable timers, and invalidate graphics rendering callbacks. Games should use this method to pause the game.
}


- (void)applicationDidEnterBackground:(UIApplication *)application {
    // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
    // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
}


- (void)applicationWillEnterForeground:(UIApplication *)application {
    // Called as part of the transition from the background to the active state; here you can undo many of the changes made on entering the background.
}


- (void)applicationDidBecomeActive:(UIApplication *)application {
    // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
}


- (void)applicationWillTerminate:(UIApplication *)application {
    // Stop any BLE discovery process that might have been taking place.
    [self stopScan];
    
    // Disconnect from the robot if necessary.
    if (peripheral)
    {
        [manager cancelPeripheralConnection:peripheral];
        [self clearPeripheral];
    }
}

// Clear BLE peripheral member.
- (void) clearPeripheral
{
    if (!peripheral)
        return;
    
    [peripheral setDelegate:nil];
    peripheral = nil;
    sendDataWriteCharacteristic = nil;
}

// Request CBCentralManager to scan for SortingHats via Nordic UART service that it broadcasts.
- (void) startScan
{
    if (!isBlePowerOn)
    {
        // Postpone the scan start until later when BLE power on is detected.
        scanOnBlePowerOn = TRUE;
        return;
    }
    else
    {
        [self updateStatus:@"Searching for Sorting Hat..."];
        [manager scanForPeripheralsWithServices:[NSArray arrayWithObject:[CBUUID UUIDWithString:@NORDIC_BLE_UART_SERVICE]] options:nil];
    }
}

// Request CBCentralManager to stop scanning for SortingHat robots.
- (void) stopScan
{
    if (!manager)
    {
        return;
    }
    
    [manager stopScan];
}

// Invoked whenever the central manager's state is updated.
- (void) centralManagerDidUpdateState:(CBCentralManager *)central
{
    NSString * state = nil;
    
    // Display an error to user if there is no BLE hardware and then force an exit.
    switch ([manager state])
    {
        case CBManagerStateUnsupported:
            state = @"The platform/hardware doesn't support Bluetooth Low Energy.";
            break;
        case CBManagerStateUnauthorized:
            state = @"The app is not authorized to use Bluetooth Low Energy.";
            break;
        case CBManagerStatePoweredOff:
            isBlePowerOn = FALSE;
            state = @"Bluetooth is currently powered off.";
            break;
        case CBManagerStatePoweredOn:
            isBlePowerOn = TRUE;
            if (scanOnBlePowerOn)
            {
                scanOnBlePowerOn = FALSE;
                [self startScan];
            }
            return;
        case CBManagerStateUnknown:
        default:
            return;
    }
    
    NSLog(@"Central manager state: %@", state);
    [self updateStatus:state];
}

- (void) updateStatus:(NSString*)string
{
    ViewController *vc = (ViewController*)[[[UIApplication sharedApplication] keyWindow] rootViewController];
    [vc updateStatus:string];
}

// Invoked when the central discovers BLE devices advertising the BLE UART service while scanning.
- (void) centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)aPeripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
    // Check for the name of the device to see if it is a Sorting Hat.
    if ([aPeripheral.name compare:@SORTING_HAT_NAME] != NSOrderedSame)
        return;

    // Connect to first Sorting Hat found.
    [self stopScan];
    peripheral = aPeripheral;
    [manager connectPeripheral:peripheral options:nil];
}

// Invoked whenever a connection is succesfully created with a Sorting Hat.
// Start discovering available BLE services on the hat.
- (void) centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)aPeripheral
{
    [aPeripheral setDelegate:self];
    [aPeripheral discoverServices:[NSArray arrayWithObject:[CBUUID UUIDWithString:@NORDIC_BLE_UART_SERVICE]]];
}

// Invoked whenever an existing connection with the peripheral is torn down.
- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)aPeripheral error:(NSError *)err
{
    NSLog(@"didDisconnectPeripheral");
    NSLog(@"err = %@", err);
    [self clearPeripheral];
    
    // Attempt to reconnect when Sorting Hat switches back on BLE radio.
    [self startScan];
}

// Invoked whenever the central manager fails to create a connection with the peripheral.
- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)aPeripheral error:(NSError *)err
{
    NSLog(@"didFailToConnectPeripheral");
    NSLog(@"err = %@", err);
    [self clearPeripheral];
}

// Invoked upon completion of a -[discoverServices:] request.
// Discover available characteristics on interested services.
- (void) peripheral:(CBPeripheral *)aPeripheral didDiscoverServices:(NSError *)error
{
    for (CBService *aService in aPeripheral.services)
    {
        /* BLE UART specific services */
        if ([aService.UUID isEqual:[CBUUID UUIDWithString:@NORDIC_BLE_UART_SERVICE]])
        {
            [aPeripheral discoverCharacteristics:nil forService:aService];
        }
    }
}

// Invoked upon completion of a -[discoverCharacteristics:forService:] request.
// Perform appropriate operations on interested characteristics.
- (void) peripheral:(CBPeripheral *)aPeripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
    /* Nordic BLE Send Data Service. */
    if ([service.UUID isEqual:[CBUUID UUIDWithString:@NORDIC_BLE_UART_SERVICE]])
    {
        for (CBCharacteristic *aChar in service.characteristics)
        {
            /* Remember Send Characteristic pointer. */
            if ([aChar.UUID isEqual:[CBUUID UUIDWithString:@NORDIC_BLE_UART_WRITE_CHARACTERISTIC]])
            {
                sendDataWriteCharacteristic = aChar;
                [self updateStatus:@"Connected to Sorting Hat."];
            }
        }
    }
}

// Send bytes to the Sorting Hat using the Nordic BLE UART service.
- (void) sendToSortingHat:(const char*)pData
{
    if (!peripheral || !sendDataWriteCharacteristic)
    {
        [self updateStatus:@"Not connected to Sorting Hat."];
        return;
    }
    
    // Send request to Sorting Hat.
    NSData* cmdData = [NSData dataWithBytes:pData length:strlen(pData)];
    [peripheral writeValue:cmdData forCharacteristic:sendDataWriteCharacteristic type:CBCharacteristicWriteWithoutResponse];
}


@end
