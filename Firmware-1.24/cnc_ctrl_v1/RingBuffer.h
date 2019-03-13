    /*This file is part of the Maslow Control Software.

    The Maslow Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Maslow Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.
    
    Copyright 2014-2017 Bar Smith*/
    
    #ifndef RingBuffer_h
    #define RingBuffer_h

    class RingBuffer{
        public:
            RingBuffer();
            int   write(char letter);
            void  print();
            char  read();
            int   length();
            int   numberOfLines();
            int   spaceAvailable();
            void  empty();
            void  readLine(String&);
            void  prettyReadLine(String&);
        private:
            void _incrementBeginning();
            int  _incrementEnd();
            void _incrementVariable(int* variable);
            int  _beginningOfString = 0;             //points to the first valid character which can be read
            int  _endOfString       = 0;             //points to the first open space which can be written
            char _buffer[INCBUFFERLENGTH];
    };

    #endif
