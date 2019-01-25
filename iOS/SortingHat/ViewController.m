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

#import "ViewController.h"
#import "AppDelegate.h"

@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
}

- (IBAction)gryffindor:(id)sender
{
    [self sendToSortingHat:"g"];
}

- (IBAction)ravenclaw:(id)sender
{
    [self sendToSortingHat:"r"];
}

- (IBAction)hufflepuff:(id)sender
{
    [self sendToSortingHat:"h"];
}

- (IBAction)slytherin:(id)sender
{
    [self sendToSortingHat:"s"];
}

-(void)updateStatus:(NSString*)string
{
    [_status setText:string];
}

- (void)sendToSortingHat:(const char*)pData
{
    if (houseSelection == pData[0])
    {
        houseSelection = '\0';
        [self updateStatus:@"Announcing house!"];
        AppDelegate* delegate = (AppDelegate*)[[UIApplication sharedApplication] delegate];
        [delegate sendToSortingHat:pData];
    }
    else
    {
        houseSelection = pData[0];
        [self updateStatus:@"Please confirm house selection..."];
    }
}

@end
