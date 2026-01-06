import React, {type ReactNode} from 'react';
import ChatKit from '../components/ChatKit';

// Root component that wraps the entire app
export default function Root({children}: {children: ReactNode}): ReactNode {
  return (
    <>
      {children}
      <ChatKit />
    </>
  );
}