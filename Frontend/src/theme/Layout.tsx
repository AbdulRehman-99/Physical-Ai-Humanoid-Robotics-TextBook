import React, {type ReactNode} from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatKit from '../components/ChatKit';

export default function Layout(props): ReactNode {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatKit />
    </>
  );
}