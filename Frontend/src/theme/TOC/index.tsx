import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import TOCItems from '../TOCItems';
import type { TOCItem } from '@docusaurus/mdx-loader';
import styles from './styles.module.css';

interface TOCProps {
  readonly toc: readonly TOCItem[];
  readonly className?: string;
}

export default function TOC({ toc, className }: TOCProps): ReactNode {
  if (!toc || toc.length === 0) {
    return null;
  }

  return (
    <div className={clsx('table-of-contents', 'thin-scrollbar', styles.tableOfContentsWrapper, className)}>
      <div className={styles.tableOfContentsCard}>
        <div className={styles.tableOfContentsHeader}>
          <h3 className={styles.tableOfContentsTitle}>On this page</h3>
        </div>
        <TOCItems toc={toc} className={styles.tableOfContents} />
      </div>
    </div>
  );
}