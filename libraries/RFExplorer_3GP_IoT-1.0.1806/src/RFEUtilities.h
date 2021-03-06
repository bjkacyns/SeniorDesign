//============================================================================
//RF Explorer 3G+ IoT for Arduino - A Spectrum Analyzer for everyone!
//Copyright ? 2010-18 Ariel Rocholl, www.rf-explorer.com
//
//This application is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public
//License as published by the Free Software Foundation; either
//version 3.0 of the License, or (at your option) any later version.
//
//This software is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//General Public License for more details.
//
//You should have received a copy of the GNU General Public
//License along with this library; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//=============================================================================


#ifndef RFEUTILITIES_H_
#define RFEUTILITIES_H_

//Compares the C string pLine with the string constant pLineCompare.
//Use for examine command
//Return 0 if is equal
boolean StartWith(char*pLine, const char* pLineCompare);

//Appends cCharacter to the pLine string, in the first pLine position. The function does not check for string length. 
//The correct pLine pointer must be set outside this function before using it for correct operation. 
//A null-character is set at the second position of the pLine string.
char *strcat(char * pLine, char cCharacter);

#endif