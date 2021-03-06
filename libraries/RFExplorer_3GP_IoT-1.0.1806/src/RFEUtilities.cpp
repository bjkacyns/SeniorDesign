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

#include "RFExplorer_3GP_IoT.h"

boolean StartWith(char*pLine, const char* pLineCompare)
{
    return (0==strncmp(pLine,pLineCompare,strlen(pLineCompare)));
}

char * strcat(char * pLine, char cCharacter)
{
    pLine[0] = cCharacter;
    pLine[1] = '\0';
    return pLine;
}