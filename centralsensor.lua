function sysCall_init()
end

function sysCall_vision(inData)
    -- callback function automatically added for backward compatibility
    -- (vision sensor have no filters anymore, but rather a callback function where image processing can be performed)
    local retVal={}
    retVal.trigger=false
    retVal.packedPackets={}
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToSensorImg(inData.handle)
    local trig,packedPacket=simVision.binaryWorkImg(inData.handle,0.500000,1.000000,0.100000,0.500000,1.000000,0.500000,1.000000,0.000000,1.570971,1.000000,true) if trig then retVal.trigger=true end if packedPacket then retVal.packedPackets[#retVal.packedPackets+1]=packedPacket end
    return retVal
end
