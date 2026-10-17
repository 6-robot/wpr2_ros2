/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2025-2035, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <driver/wpr2_driver.h>
#include <math.h>

static bool bFirstQuart = true;

CWPR2_driver::CWPR2_driver()
{
   	m_SendBuf = new unsigned char[1024];
	memset(m_SendBuf, 0, 1024);
	memset(m_ParseBuf, 0, 128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 14;
	nBaseMode = MODE_OMNI;

	for (int i = 0; i < 4; i++)
	{
		arValAD[i] = 0;
	}
	for (int i = 0; i < 4; i++)
	{
		arMotorCurrent[i] = 0;
		arMotorPos[i] = 0;
	}
	for (int i = 0; i < 15; i++)
	{
		arJointSpeed[i] = 2000;
		arJointPos[i] = 0;
		arJointCurrentRecv[i] = 0;
		arJointPosRecv[i] = 0;
		arJointPosLastRecv[i] = 0;
	}
	arJointSpeed[0] = 18000;
	arJointPos[2] = -9000;
	arJointPos[5] = -9000;
	arJointPos[9] = 9000;
	arJointPos[12] = 9000;
	arJointPos[7] = 10000;
	arJointPos[14] = 10000;
	fVoltage = 0;
	nParseCount = 0;
	fQuatW = 0;
	fQuatX = 0;
	fQuatY = 0;
	fQuatZ = 0;
	
	fGyroX = 0;
	fGyroY = 0;
	fGyroZ = 0;
	
	fAccX = 0;
	fAccY = 0;
	fAccZ = 0;

	fCurYaw = 0;
	fFirstYaw = 0;
	bCalFirstYaw = false; 

	fLinearAccLimit = 0.2;
	fAngularAccLimit = 0.1;
}
    
CWPR2_driver::~CWPR2_driver()
{
	delete []m_SendBuf;
}

void CWPR2_driver::Parse(unsigned char inData)
{
    if (!m_bFrameStart) {
        if (m_lastRecv == 0x55 && inData == 0xaa) {
            m_ParseBuf[0] = 0x55;
            m_ParseBuf[1] = 0xaa;
            m_nRecvIndex = 2;
            m_bFrameStart = true;
            m_lastRecv = 0;
        } else {
            m_lastRecv = inData;
        }
        return;
    }
    m_ParseBuf[m_nRecvIndex++] = inData;
    if (m_nRecvIndex == 4) {
        m_nFrameLength = inData + 8;
        if (m_nFrameLength > 256) {
            m_bFrameStart = false;
            m_nRecvIndex = 0;
            m_lastRecv = inData;
            return;
        }
    }
    if (m_nRecvIndex == m_nFrameLength) {
        m_ParseFrame();
        m_bFrameStart = false;
        m_nRecvIndex = 0;
        m_lastRecv = 0;
    }
}

void CWPR2_driver::m_Split2Bytes(unsigned char *inTarg, short inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned short temp;
	memcpy(&temp, &inSrc, sizeof(short));
	inTarg[1] = (unsigned char)temp & 0x00ff;

	temp >>= 8;

	inTarg[0] = (unsigned char)temp & 0x00ff;
}

void CWPR2_driver::m_Split4Bytes(unsigned char *inTarg, int inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned int temp;
	memcpy(&temp, &inSrc, sizeof(int));
	inTarg[3] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[2] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[1] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[0] = (unsigned char)temp & 0x00ff;
}

short CWPR2_driver::m_WordFromChar(unsigned char *inBuf)
{
	static short wtemp;
	wtemp = 0;
	wtemp = *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}

int CWPR2_driver::m_IntFromChar(unsigned char *inBuf)
{
	static int itemp;
	itemp = 0;
	itemp = *(inBuf);

	itemp <<= 8;
	itemp |= *(inBuf + 1);

	itemp <<= 8;
	itemp |= *(inBuf + 2);

	itemp <<= 8;
	itemp |= *(inBuf + 3);

	return itemp;
}

void CWPR2_driver::m_CalSendSum(unsigned char* pNewCmdBuf)
{
	int nLen = pNewCmdBuf[3] + 7;

	pNewCmdBuf[nLen - 1] = 0x00;
	for (int i = 0; i < nLen - 1; i++)
	{
		pNewCmdBuf[nLen - 1] += pNewCmdBuf[i];
	}
}

void CWPR2_driver::m_ParseFrame()
{
	nParseCount = 0;
	if (m_ParseBuf[4] == 0x01 && m_nFrameLength >= 10)	//电压
	{
		short wValue = m_WordFromChar(&m_ParseBuf[7]);
		fVoltage = (float)wValue * 0.01;
	}
	if (m_ParseBuf[4] == 0x06 && m_nFrameLength >= 10)	//IO
	{
		for (int i = 0; i < 4; i++)
		{
			unsigned char tmp = 0x01;
			tmp = tmp<<i;
			tmp = tmp & m_ParseBuf[8];
			if (tmp == 0)
			{
				arValIOInput[i] = 0;
			} 
			else
			{
				arValIOInput[i] = 1;
			}
		}
	}

	if (m_ParseBuf[4] == 0x07 && m_nFrameLength >= 16)	//AD
	{
		//AD 0~4
		for (int i = 0; i < 4; i++)
		{
			arValAD[i] = m_WordFromChar(&m_ParseBuf[7 + i * 2]);
		}
	}

	if (m_ParseBuf[4] == 0x08)	//电机模块
	{
		if (m_ParseBuf[5] == 0x60 && m_nFrameLength >= 17)	//底盘电机
		{
			int nCurMotorID = m_ParseBuf[7] - 1;
			if (nCurMotorID >= 0 && nCurMotorID < 4)
			{
				arMotorCurrent[nCurMotorID] = m_IntFromChar(&m_ParseBuf[8]);
				arMotorPos[nCurMotorID] = m_IntFromChar(&m_ParseBuf[12]);
			}
			else
			{
				//id超限
			}
		}
		if (m_ParseBuf[5] == 0x63 && m_nFrameLength >= 128)	//上肢电机信息
		{
			for(int i=0;i<15;i++)
			{
				arJointCurrentRecv[i] = m_IntFromChar(&m_ParseBuf[7+8*i]);
				arJointPosRecv[i] = m_IntFromChar(&m_ParseBuf[7+4+8*i]);
				//printf("接收的[%d]关节位置数值为 %d\n", i, arJointPosRecv[i] );
			}
			has_joint_feedback = true;
		}
	}

	if (m_ParseBuf[4] == 0x09 && m_nFrameLength >= 16)	//IMU
	{
		if(m_ParseBuf[6] == 0x01 && m_nFrameLength >= 20)	//GYRO
		{
			fGyroX = (float)m_Piece2int(&m_ParseBuf[7]);
			fGyroY = (float)m_Piece2int(&m_ParseBuf[11]);
			fGyroZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x02 && m_nFrameLength >= 20)	//ACC
		{
			fAccX = (float)m_Piece2int(&m_ParseBuf[7]);
			fAccY = (float)m_Piece2int(&m_ParseBuf[11]);
			fAccZ = (float)m_Piece2int(&m_ParseBuf[15]);
		}
		if(m_ParseBuf[6] == 0x03)	//QUAT-W-X
		{
			fQuatW = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatX = (float)m_Piece2int(&m_ParseBuf[11]);
		}
		if(m_ParseBuf[6] == 0x04)	//QUAT-Y-Z
		{
			fQuatY = (float)m_Piece2int(&m_ParseBuf[7]);
			fQuatZ = (float)m_Piece2int(&m_ParseBuf[11]);
			// yaw: (about Z axis)
    		//fCurYaw = atan2(2*fQuatX*fQuatY - 2*fQuatW*fQuatZ, 2*fQuatW*fQuatW + 2*fQuatX*fQuatX - 1);
			//printf("[CWPR2_driver] fYaw = %.2f\n",fCurYaw);
			if(bFirstQuart == true)
			{
				//fFirstYaw = fCurYaw;
				bCalFirstYaw = true;
				bFirstQuart = false;
			}
		}
	}
}

void CWPR2_driver::m_DisRecv()
{
	
}

int CWPR2_driver::GenCmd(int inBuffOffset, int inDevID, int inModule, int inMethod, unsigned char* inData, int inDataLen)
{
	int nCmdLen = 0;

	int nTailIndex = inBuffOffset + 7 + inDataLen;
	if (nTailIndex >= 1024)
	{
		return nCmdLen;
	}

	unsigned char* pNewCmd = m_SendBuf + inBuffOffset;
	pNewCmd[0] = 0x55;
	pNewCmd[1] = 0xaa;
	pNewCmd[2] = (unsigned char)inDevID;
	pNewCmd[3] = (unsigned char)inDataLen;
	pNewCmd[4] = (unsigned char)inModule;
	pNewCmd[5] = (unsigned char)inMethod;
	memcpy(&pNewCmd[6], inData, inDataLen);

	m_CalSendSum(pNewCmd);

	nCmdLen = inDataLen + 7;
	return nCmdLen;
}

void CWPR2_driver::SendMotors(int inMotor1, int inMotor2, int inMotor3, int inMotor4)
{
	static unsigned char arMotorSpeedData[16];
	m_Split4Bytes(arMotorSpeedData, inMotor1);
	m_Split4Bytes(arMotorSpeedData + 4, inMotor2);
	m_Split4Bytes(arMotorSpeedData + 8, inMotor3);
	m_Split4Bytes(arMotorSpeedData + 12, inMotor4);
	int nCmdLenght = GenCmd(0, 0x41, 0x08, 0x60, arMotorSpeedData, 16);
	Send(m_SendBuf, nCmdLenght);
}

void CWPR2_driver::SendLED(int inMode,  unsigned char inR,  unsigned char inG,  unsigned char inB)
{
	unsigned char arLEDData[4];
	arLEDData[0] = inMode;
	arLEDData[1] = inR;
	arLEDData[2] = inG;
	arLEDData[3] = inB;
	int nCmdLenght = GenCmd(0, 0x41, 0x0a, 0x70, arLEDData, 4);
	Send(m_SendBuf, nCmdLenght);
}

void CWPR2_driver::SendOutput(int inOut1, int inOut0)
{
	unsigned char arOutputData = 0;
	if (inOut1 > 0)
	{
		arOutputData = arOutputData | 0x02;
	}
	if (inOut0 > 0)
	{
		arOutputData = arOutputData | 0x01;
	}
	int nCmdLenght = GenCmd(0, 0x41, 0x0a, 0x70, &arOutputData, 1);
	Send(m_SendBuf, nCmdLenght);
}

void CWPR2_driver::SetTorsoHeight(int inPos, int inSpeed)
{
	arJointPos[0] = inPos;
	arJointSpeed[0] = inSpeed;
}
void CWPR2_driver::SetLeftArm(int* inPos, int* inSpeed)
{
	for(int i=0;i<6;i++)
	{
		arJointPos[i+1] = inPos[i];
		arJointSpeed[i+1] = inSpeed[i];
	}
}
void CWPR2_driver::SetLeftGripper(int inPos, int inSpeed)
{
	arJointPos[7] = inPos;
	arJointSpeed[7] = inSpeed;
}
void CWPR2_driver::SetRightArm(int* inPos, int* inSpeed)
{
	for(int i=0;i<6;i++)
	{
		arJointPos[i+8] = inPos[i];
		arJointSpeed[i+8] = inSpeed[i];
	}
}
void CWPR2_driver::SetRightGripper(int inPos, int inSpeed)
{
	arJointPos[14] = inPos;
	arJointSpeed[14] = inSpeed;
}

void CWPR2_driver::JointAction()
{
	TorsoCmd(arJointPos,arJointSpeed);
}

int SpeedFixed(int inValue)
{
	if(inValue > 6000)
		return 6000;
	if(inValue < 0)
		return 0;
	return inValue;
}

// 麦克纳姆轮参数
static float fMecanumLinearMotorKX = 1890;
static float fMecanumLinearMotorKY = 2130;
static float fMecanumAngularMotorK = 765;
// 四轮全向参数
static float fOmniLinearMotorKX = 1415;
static float fOmniLinearMotorKY = 1415;
static float fOmniAngularMotorK = 528;
void CWPR2_driver::Velocity(float inX, float inY, float inAngular)
{
	int nMotorToSend[4] = {};

	if(nBaseMode == MODE_MECANUM)
	{
		// 麦克纳姆轮
		//upward backward
		int nTmpMotorVal = inX * fMecanumLinearMotorKX;
		nMotorToSend[0] = nTmpMotorVal;
		nMotorToSend[1] = -nTmpMotorVal;
		nMotorToSend[2] = -nTmpMotorVal;
		nMotorToSend[3] = nTmpMotorVal;

		//shif left right
		nTmpMotorVal = inY * fMecanumLinearMotorKY;
		nMotorToSend[0] += -nTmpMotorVal;
		nMotorToSend[1] += -nTmpMotorVal;
		nMotorToSend[2] += nTmpMotorVal;
		nMotorToSend[3] += nTmpMotorVal;

		//Turning 
		nTmpMotorVal = inAngular * fMecanumAngularMotorK;
		nMotorToSend[0] += -nTmpMotorVal;
		nMotorToSend[1] += -nTmpMotorVal;
		nMotorToSend[2] += -nTmpMotorVal;
		nMotorToSend[3] += -nTmpMotorVal;
	}
	
	if(nBaseMode == MODE_OMNI)
	{
		// 四轮全向
		//upward backward
		int nTmpMotorVal = inX * fOmniLinearMotorKX;
		nMotorToSend[0] = nTmpMotorVal;
		nMotorToSend[1] = -nTmpMotorVal;
		nMotorToSend[2] = -nTmpMotorVal;
		nMotorToSend[3] = nTmpMotorVal;

		//shif left right
		nTmpMotorVal = inY * fOmniLinearMotorKY;
		nMotorToSend[0] += -nTmpMotorVal;
		nMotorToSend[1] += -nTmpMotorVal;
		nMotorToSend[2] += nTmpMotorVal;
		nMotorToSend[3] += nTmpMotorVal;

		//Turning 
		nTmpMotorVal = inAngular * fOmniAngularMotorK;
		nMotorToSend[0] += -nTmpMotorVal;
		nMotorToSend[1] += -nTmpMotorVal;
		nMotorToSend[2] += -nTmpMotorVal;
		nMotorToSend[3] += -nTmpMotorVal;
	}

	//printf("[CWPB_driver::Mecanum]-> [0]%d [1]%d [2]%d [3]%d \n", nMotorToSend[0], nMotorToSend[1], nMotorToSend[2], nMotorToSend[3]);

	SendMotors(nMotorToSend[0],nMotorToSend[1],nMotorToSend[2],nMotorToSend[3]);
}

float CWPR2_driver::GetYaw()
{
	float diffYaw = fCurYaw - fFirstYaw;
	return diffYaw;
}

void CWPR2_driver::MotorCmd(int inMethod, int inID1, int inValue1, int inID2, int inValue2)
{
	static unsigned char arMotorSpeedData[12];
	m_Split2Bytes(arMotorSpeedData,inID1);
	m_Split4Bytes(arMotorSpeedData + 2, inValue1);

	m_Split2Bytes(arMotorSpeedData + 6, inID2);
	m_Split4Bytes(arMotorSpeedData + 8, inValue2);
	int nCmdLenght = GenCmd(0, 0x40, 0x08, inMethod, arMotorSpeedData, 12);
	Send(m_SendBuf, nCmdLenght);
}


void CWPR2_driver::TorsoCmd(int* inPos, int* inSpeed)
{
	static unsigned char arTorsoCmdData[15*8];
	for(int i=0;i<15;i++)
	{
		m_Split4Bytes(arTorsoCmdData + i*8, inSpeed[i]);
		m_Split4Bytes(arTorsoCmdData + i*8 + 4, inPos[i]);
		// printf("发送[%d]关节位置数值为 %d\n", i, inPos[i]);
	}
	int nCmdLenght = GenCmd(0, 0x41, 0x08, 0x63, arTorsoCmdData, 15*8);
	Send(m_SendBuf, nCmdLenght);
}

bool CWPR2_driver::JointsArrived()
{
	bool bArrived = true;
	for(int i=0;i<15;i++)
	{
		if(abs (arJointPosRecv[i] - arJointPosLastRecv[i]) > 100 )
		{
			bArrived = false;
		}
		arJointPosLastRecv[i] = arJointPosRecv[i];
	}
	return bArrived;
}
