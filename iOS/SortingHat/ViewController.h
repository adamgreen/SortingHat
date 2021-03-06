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

@interface ViewController : UIViewController
{
    uint8_t houseSelection;
}

@property (weak, nonatomic) IBOutlet UILabel *status;

- (IBAction)gryffindor:(id)sender;
- (IBAction)ravenclaw:(id)sender;
- (IBAction)hufflepuff:(id)sender;
- (IBAction)slytherin:(id)sender;

- (void)updateStatus:(NSString*)string;
- (void)sendToSortingHat:(const char*)pData;

@end

